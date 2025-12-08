package com.example.deadr.slam

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Matrix
import android.util.Log
import android.util.Size
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

/**
 * Camera manager for SLAM using CameraX.
 * Handles camera lifecycle and frame processing for visual odometry.
 * 
 * Now uses OpenCV ORB-based visual odometry for accurate motion estimation
 * on non-ARCore devices.
 */
class CameraManager(private val context: Context) {
    
    companion object {
        private const val TAG = "CameraManager"
        private const val ANALYSIS_WIDTH = 640
        private const val ANALYSIS_HEIGHT = 480
    }
    
    private var cameraProvider: ProcessCameraProvider? = null
    private var imageAnalyzer: ImageAnalysis? = null
    private var cameraExecutor: ExecutorService? = null
    
    // LAZY initialization of ORB visual odometry - only created after OpenCV is loaded
    private var orbVisualOdometry: OrbVisualOdometry? = null
    
    // State flows for UI observation
    private val _isRunning = MutableStateFlow(false)
    val isRunning: StateFlow<Boolean> = _isRunning.asStateFlow()
    
    private val _currentFeatures = MutableStateFlow<List<VisualOdometry.FeaturePoint>>(emptyList())
    val currentFeatures: StateFlow<List<VisualOdometry.FeaturePoint>> = _currentFeatures.asStateFlow()
    
    private val _lastMotionEstimate = MutableStateFlow<VisualOdometry.MotionEstimate?>(null)
    val lastMotionEstimate: StateFlow<VisualOdometry.MotionEstimate?> = _lastMotionEstimate.asStateFlow()
    
    private val _frameCount = MutableStateFlow(0)
    val frameCount: StateFlow<Int> = _frameCount.asStateFlow()
    
    // Current pose from visual odometry
    private val _currentPose = MutableStateFlow(VisualOdometry.Pose())
    val currentPose: StateFlow<VisualOdometry.Pose> = _currentPose.asStateFlow()
    
    // Landmark count
    private val _landmarkCount = MutableStateFlow(0)
    val landmarkCount: StateFlow<Int> = _landmarkCount.asStateFlow()
    
    // FPS tracking
    private var lastFrameTime = 0L
    private val _fps = MutableStateFlow(0)
    val fps: StateFlow<Int> = _fps.asStateFlow()
    
    // OpenCV initialization status
    private val _isOpenCVReady = MutableStateFlow(false)
    val isOpenCVReady: StateFlow<Boolean> = _isOpenCVReady.asStateFlow()
    
    // Callback for motion updates
    var onMotionEstimate: ((VisualOdometry.MotionEstimate) -> Unit)? = null
    
    init {
        // Initialize OpenCV on background thread - do NOT create any OpenCV objects before this
        Executors.newSingleThreadExecutor().execute {
            try {
                val initialized = OrbFeatureDetector.initOpenCV()
                if (initialized) {
                    // Only create ORB visual odometry AFTER OpenCV is loaded
                    orbVisualOdometry = OrbVisualOdometry()
                    Log.d(TAG, "OpenCV and ORB VO initialized successfully")
                } else {
                    Log.e(TAG, "OpenCV initialization failed - ORB features will not work")
                }
                _isOpenCVReady.value = initialized
            } catch (e: Exception) {
                Log.e(TAG, "Failed to initialize OpenCV/ORB", e)
                _isOpenCVReady.value = false
            }
        }
    }
    
    /**
     * Start the camera for SLAM processing.
     * @param lifecycleOwner Activity or Fragment lifecycle owner
     * @param previewView Optional PreviewView for camera preview display
     */
    fun startCamera(
        lifecycleOwner: LifecycleOwner,
        previewView: PreviewView? = null
    ) {
        cameraExecutor = Executors.newSingleThreadExecutor()
        
        val cameraProviderFuture = ProcessCameraProvider.getInstance(context)
        
        cameraProviderFuture.addListener({
            try {
                cameraProvider = cameraProviderFuture.get()
                bindCameraUseCases(lifecycleOwner, previewView)
                _isRunning.value = true
            } catch (e: Exception) {
                Log.e(TAG, "Failed to start camera", e)
            }
        }, ContextCompat.getMainExecutor(context))
    }
    
    private fun bindCameraUseCases(
        lifecycleOwner: LifecycleOwner,
        previewView: PreviewView?
    ) {
        val provider = cameraProvider ?: return
        
        // Unbind any existing use cases
        provider.unbindAll()
        
        // Camera selector - use back camera
        val cameraSelector = CameraSelector.Builder()
            .requireLensFacing(CameraSelector.LENS_FACING_BACK)
            .build()
        
        // Preview use case (optional)
        val preview = previewView?.let { view ->
            Preview.Builder()
                .build()
                .also { preview ->
                    preview.setSurfaceProvider(view.surfaceProvider)
                }
        }
        
        // Image analysis for SLAM
        imageAnalyzer = ImageAnalysis.Builder()
            .setTargetResolution(Size(ANALYSIS_WIDTH, ANALYSIS_HEIGHT))
            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
            .build()
            .also { analysis ->
                analysis.setAnalyzer(cameraExecutor!!) { imageProxy ->
                    processFrame(imageProxy)
                }
            }
        
        try {
            // Bind use cases to camera
            if (preview != null) {
                provider.bindToLifecycle(
                    lifecycleOwner,
                    cameraSelector,
                    preview,
                    imageAnalyzer
                )
            } else {
                provider.bindToLifecycle(
                    lifecycleOwner,
                    cameraSelector,
                    imageAnalyzer
                )
            }
        } catch (e: Exception) {
            Log.e(TAG, "Camera binding failed", e)
        }
    }
    
    private fun processFrame(imageProxy: ImageProxy) {
        try {
            // Skip if OpenCV not ready or VO not initialized
            val vo = orbVisualOdometry
            if (!_isOpenCVReady.value || vo == null) {
                imageProxy.close()
                return
            }
            
            // Calculate FPS
            val currentTime = System.currentTimeMillis()
            if (lastFrameTime > 0) {
                val dt = currentTime - lastFrameTime
                if (dt > 0) {
                    _fps.value = (1000 / dt).toInt()
                }
            }
            lastFrameTime = currentTime
            
            // Convert ImageProxy to Bitmap for processing
            val bitmap = imageProxy.toBitmap()
            
            // Rotate based on image rotation
            val rotatedBitmap = rotateBitmap(bitmap, imageProxy.imageInfo.rotationDegrees.toFloat())
            
            // Process with ORB visual odometry
            val orbMotion = vo.processFrame(rotatedBitmap)
            
            // Update state
            _frameCount.value++
            
            // Convert ORB features to compatible format
            val orbFeatures = vo.getCurrentFeatures()
            _currentFeatures.value = orbFeatures.map { f ->
                VisualOdometry.FeaturePoint(
                    x = f.x,
                    y = f.y,
                    score = f.score,
                    type = if (f.type == OrbVisualOdometry.FeatureType.OBSTACLE) 
                        VisualOdometry.FeatureType.OBSTACLE 
                    else 
                        VisualOdometry.FeatureType.ENVIRONMENT
                )
            }
            
            // Convert ORB pose to compatible format
            val orbPose = vo.getCurrentPose()
            _currentPose.value = VisualOdometry.Pose(orbPose.x, orbPose.y, orbPose.heading)
            
            _landmarkCount.value = vo.getLandmarkCount()
            
            orbMotion?.let { motion ->
                // Convert to compatible MotionEstimate format
                val compatibleMotion = VisualOdometry.MotionEstimate(
                    deltaX = motion.deltaX,
                    deltaY = motion.deltaY,
                    rotation = motion.rotation,
                    confidence = motion.confidence,
                    matchCount = motion.matchCount
                )
                _lastMotionEstimate.value = compatibleMotion
                onMotionEstimate?.invoke(compatibleMotion)
            }
            
            // Clean up
            if (rotatedBitmap != bitmap) {
                rotatedBitmap.recycle()
            }
            bitmap.recycle()
            
        } catch (e: Exception) {
            Log.e(TAG, "Frame processing error", e)
        } finally {
            imageProxy.close()
        }
    }
    
    private fun rotateBitmap(bitmap: Bitmap, degrees: Float): Bitmap {
        if (degrees == 0f) return bitmap
        
        val matrix = Matrix().apply {
            postRotate(degrees)
        }
        return Bitmap.createBitmap(bitmap, 0, 0, bitmap.width, bitmap.height, matrix, true)
    }
    
    /**
     * Stop the camera and release resources.
     */
    fun stopCamera() {
        try {
            cameraProvider?.unbindAll()
            cameraExecutor?.shutdown()
            orbVisualOdometry?.reset()
            _isRunning.value = false
            _frameCount.value = 0
            _currentFeatures.value = emptyList()
            _lastMotionEstimate.value = null
            _currentPose.value = VisualOdometry.Pose()
            _landmarkCount.value = 0
            _fps.value = 0
            lastFrameTime = 0L
        } catch (e: Exception) {
            Log.e(TAG, "Error stopping camera", e)
        }
    }
    
    /**
     * Reset visual odometry state without stopping camera.
     */
    fun resetOdometry() {
        orbVisualOdometry?.reset()
        _frameCount.value = 0
        _lastMotionEstimate.value = null
        _currentPose.value = VisualOdometry.Pose()
        _landmarkCount.value = 0
    }
    
    /**
     * Get visual odometry instance for direct access.
     * @deprecated Use the state flows instead
     */
    fun getVisualOdometry(): VisualOdometry {
        // Return a wrapper that delegates to orbVisualOdometry
        return VisualOdometry()
    }
    
    /**
     * Release all resources. Call when done with camera.
     */
    fun release() {
        stopCamera()
        orbVisualOdometry?.release()
        orbVisualOdometry = null
    }
}
