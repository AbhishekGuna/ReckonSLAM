package com.example.deadr.slam

import android.graphics.Bitmap
import android.util.Log
import org.opencv.android.OpenCVLoader
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfKeyPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.features2d.ORB
import org.opencv.imgproc.Imgproc

/**
 * ORB Feature Detector using OpenCV.
 * Detects ORB keypoints and computes binary descriptors for feature matching.
 */
class OrbFeatureDetector {
    
    companion object {
        private const val TAG = "OrbFeatureDetector"
        
        // ORB parameters optimized for mobile
        private const val N_FEATURES = 500           // Max features to detect
        private const val SCALE_FACTOR = 1.2f        // Pyramid scale factor
        private const val N_LEVELS = 8               // Number of pyramid levels
        private const val EDGE_THRESHOLD = 31        // Size of border where features are not detected
        private const val FIRST_LEVEL = 0            // First level of pyramid
        private const val WTA_K = 2                  // Points to compute oriented BRIEF descriptor
        private const val PATCH_SIZE = 31            // Size of patch used for descriptor
        private const val FAST_THRESHOLD = 20        // FAST detector threshold
        
        @Volatile
        private var isOpenCVInitialized = false
        
        /**
         * Initialize OpenCV. Must be called before using this class.
         */
        fun initOpenCV(): Boolean {
            if (!isOpenCVInitialized) {
                isOpenCVInitialized = OpenCVLoader.initLocal()
                if (isOpenCVInitialized) {
                    Log.d(TAG, "OpenCV initialized successfully: ${Core.getVersionString()}")
                } else {
                    Log.e(TAG, "OpenCV initialization failed")
                }
            }
            return isOpenCVInitialized
        }
    }
    
    // ORB detector instance
    private var orbDetector: ORB? = null
    
    // Reusable Mat objects to avoid allocation per frame
    private var grayMat: Mat? = null
    private var rgbMat: Mat? = null
    
    /**
     * Detected keypoint with descriptor.
     */
    data class OrbKeypoint(
        val x: Float,
        val y: Float,
        val size: Float,
        val angle: Float,
        val response: Float,
        val octave: Int
    )
    
    /**
     * Result of feature detection.
     */
    data class DetectionResult(
        val keypoints: List<OrbKeypoint>,
        val descriptors: Mat,
        val keypointsMat: MatOfKeyPoint
    )
    
    init {
        if (isOpenCVInitialized) {
            createDetector()
        }
    }
    
    private fun createDetector() {
        try {
            orbDetector = ORB.create(
                N_FEATURES,
                SCALE_FACTOR,
                N_LEVELS,
                EDGE_THRESHOLD,
                FIRST_LEVEL,
                WTA_K,
                ORB.HARRIS_SCORE,
                PATCH_SIZE,
                FAST_THRESHOLD
            )
            Log.d(TAG, "ORB detector created successfully")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to create ORB detector", e)
        }
    }
    
    /**
     * Detect ORB features in a bitmap.
     * @param bitmap Input image
     * @return DetectionResult with keypoints and descriptors, or null on failure
     */
    fun detect(bitmap: Bitmap): DetectionResult? {
        if (!isOpenCVInitialized) {
            if (!initOpenCV()) {
                Log.e(TAG, "OpenCV not initialized")
                return null
            }
            createDetector()
        }
        
        val detector = orbDetector ?: run {
            Log.e(TAG, "ORB detector not available")
            return null
        }
        
        try {
            // Convert Bitmap to Mat
            if (rgbMat == null) {
                rgbMat = Mat()
            }
            Utils.bitmapToMat(bitmap, rgbMat)
            
            // Convert to grayscale
            if (grayMat == null) {
                grayMat = Mat()
            }
            Imgproc.cvtColor(rgbMat, grayMat, Imgproc.COLOR_RGBA2GRAY)
            
            // Detect keypoints
            val keypoints = MatOfKeyPoint()
            val descriptors = Mat()
            
            detector.detectAndCompute(grayMat, Mat(), keypoints, descriptors)
            
            // Convert to our data class
            val keypointList = keypoints.toList().map { kp ->
                OrbKeypoint(
                    x = kp.pt.x.toFloat(),
                    y = kp.pt.y.toFloat(),
                    size = kp.size.toFloat(),
                    angle = kp.angle.toFloat(),
                    response = kp.response.toFloat(),
                    octave = kp.octave
                )
            }
            
            return DetectionResult(keypointList, descriptors, keypoints)
            
        } catch (e: Exception) {
            Log.e(TAG, "Feature detection failed", e)
            return null
        }
    }
    
    /**
     * Release resources.
     */
    fun release() {
        grayMat?.release()
        rgbMat?.release()
        grayMat = null
        rgbMat = null
    }
}
