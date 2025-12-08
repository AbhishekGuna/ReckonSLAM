package com.example.deadr.slam

import android.graphics.Bitmap
import android.util.Log
import org.opencv.calib3d.Calib3d
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfByte
import org.opencv.core.MatOfKeyPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * ORB-based Visual Odometry with proper geometric pose estimation.
 * Uses Essential Matrix decomposition for accurate motion estimation.
 */
class OrbVisualOdometry {
    
    companion object {
        private const val TAG = "OrbVisualOdometry"
        
        // Camera intrinsics (approximate for typical smartphone)
        // These should ideally be calibrated per-device
        private const val FOCAL_LENGTH = 500.0      // Approximate focal length in pixels
        private const val CX = 320.0                // Principal point X (image center)
        private const val CY = 240.0                // Principal point Y (image center)
        
        // RANSAC parameters for Essential Matrix
        private const val RANSAC_THRESHOLD = 1.0    // Pixel threshold for RANSAC
        private const val RANSAC_CONFIDENCE = 0.999 // Confidence level
        
        // Pose estimation parameters
        private const val MIN_MATCHES = 15          // Minimum matches for pose estimation
        private const val MIN_INLIERS = 8           // Minimum inliers after RANSAC
        private const val PIXEL_TO_METER = 0.002f   // Scale factor (tuned for walking speed)
    }
    
    // Feature detector and matcher
    private val featureDetector = OrbFeatureDetector()
    private val featureMatcher = OrbFeatureMatcher()
    
    // Previous frame data - store as Mat (descriptors) and separate keypoint list
    private var prevDescriptors: Mat? = null
    private var prevKeypointsMat: Mat? = null  // Store the underlying Mat
    private var prevKeypointList: List<OrbFeatureDetector.OrbKeypoint> = emptyList()
    
    // Camera intrinsic matrix
    private val cameraMatrix: Mat by lazy {
        Mat(3, 3, CvType.CV_64FC1).apply {
            put(0, 0, FOCAL_LENGTH)
            put(0, 1, 0.0)
            put(0, 2, CX)
            put(1, 0, 0.0)
            put(1, 1, FOCAL_LENGTH)
            put(1, 2, CY)
            put(2, 0, 0.0)
            put(2, 1, 0.0)
            put(2, 2, 1.0)
        }
    }
    
    // Current pose in world coordinates
    private val currentPose = Pose()
    
    // Landmarks for mapping
    private val landmarks = mutableListOf<Landmark>()
    private var landmarkIdCounter = 0L
    
    /**
     * Feature point compatible with existing VisualOdometry interface.
     */
    data class FeaturePoint(
        val x: Float,
        val y: Float,
        val score: Float,
        val type: FeatureType = FeatureType.OBSTACLE
    )
    
    enum class FeatureType {
        OBSTACLE,
        ENVIRONMENT
    }
    
    /**
     * Motion estimate from visual odometry.
     */
    data class MotionEstimate(
        val deltaX: Float,
        val deltaY: Float,
        val rotation: Float,
        val confidence: Float,
        val matchCount: Int,
        val inlierCount: Int = 0
    )
    
    /**
     * Camera pose.
     */
    data class Pose(
        var x: Float = 0f,
        var y: Float = 0f,
        var heading: Float = 0f
    )
    
    /**
     * Map landmark.
     */
    data class Landmark(
        val id: Long,
        var x: Float,
        var y: Float,
        var quality: Int = 1
    )
    
    /**
     * Process a new camera frame and estimate motion.
     */
    fun processFrame(frame: Bitmap): MotionEstimate? {
        // Detect features
        val detection = featureDetector.detect(frame) ?: run {
            Log.w(TAG, "Feature detection failed")
            return null
        }
        
        // First frame - just store and return
        if (prevDescriptors == null) {
            prevDescriptors = detection.descriptors.clone()
            prevKeypointsMat = detection.keypointsMat.clone()
            prevKeypointList = detection.keypoints
            return null
        }
        
        // Create MatOfKeyPoint from stored Mat for matching
        val prevKeypoints = MatOfKeyPoint(prevKeypointsMat)
        
        // Match features
        val matchResult = featureMatcher.match(
            prevDescriptors!!,
            prevKeypoints,
            detection.descriptors,
            detection.keypointsMat
        )
        
        if (matchResult == null || matchResult.matches.size < MIN_MATCHES) {
            Log.d(TAG, "Not enough matches: ${matchResult?.matches?.size ?: 0}")
            // Update previous frame anyway to avoid getting stuck
            prevDescriptors?.release()
            prevKeypointsMat?.release()
            prevDescriptors = detection.descriptors.clone()
            prevKeypointsMat = detection.keypointsMat.clone()
            prevKeypointList = detection.keypoints
            return MotionEstimate(0f, 0f, 0f, 0f, matchResult?.matches?.size ?: 0, 0)
        }
        
        // Estimate pose from matches
        val motionEstimate = estimatePose(matchResult.matches, frame.width, frame.height)
        
        // Update global pose if confidence is sufficient
        if (motionEstimate != null && motionEstimate.confidence > 0.15f) {
            updatePose(motionEstimate)
        }
        
        // Update previous frame
        prevDescriptors?.release()
        prevKeypointsMat?.release()
        prevDescriptors = detection.descriptors.clone()
        prevKeypointsMat = detection.keypointsMat.clone()
        prevKeypointList = detection.keypoints
        
        return motionEstimate
    }
    
    /**
     * Estimate camera motion using Essential Matrix decomposition.
     */
    private fun estimatePose(
        matches: List<OrbFeatureMatcher.FeatureMatch>,
        width: Int,
        height: Int
    ): MotionEstimate? {
        if (matches.size < MIN_MATCHES) {
            return MotionEstimate(0f, 0f, 0f, 0f, matches.size, 0)
        }
        
        try {
            // Build point arrays for Essential Matrix estimation
            val prevPoints = MatOfPoint2f()
            val currPoints = MatOfPoint2f()
            
            val prevPointList = matches.map { Point(it.queryX.toDouble(), it.queryY.toDouble()) }
            val currPointList = matches.map { Point(it.trainX.toDouble(), it.trainY.toDouble()) }
            
            prevPoints.fromList(prevPointList)
            currPoints.fromList(currPointList)
            
            // Find Essential Matrix using RANSAC
            // OpenCV 4.9 signature: findEssentialMat(points1, points2, cameraMatrix, method, prob, threshold, maxIters, mask)
            val mask = Mat()
            val essentialMatrix = Calib3d.findEssentialMat(
                prevPoints,
                currPoints,
                cameraMatrix,
                Calib3d.RANSAC,
                RANSAC_CONFIDENCE,
                RANSAC_THRESHOLD,
                1000,  // maxIters
                mask
            )
            
            if (essentialMatrix.rows() == 0 || essentialMatrix.cols() == 0) {
                Log.w(TAG, "Essential Matrix estimation failed")
                prevPoints.release()
                currPoints.release()
                mask.release()
                return MotionEstimate(0f, 0f, 0f, 0.1f, matches.size, 0)
            }
            
            // Count inliers from mask
            var inlierCount = 0
            if (mask.rows() > 0) {
                for (i in 0 until mask.rows()) {
                    val value = mask.get(i, 0)
                    if (value != null && value.isNotEmpty() && value[0] != 0.0) {
                        inlierCount++
                    }
                }
            }
            
            if (inlierCount < MIN_INLIERS) {
                Log.d(TAG, "Too few inliers: $inlierCount")
                prevPoints.release()
                currPoints.release()
                essentialMatrix.release()
                mask.release()
                return MotionEstimate(0f, 0f, 0f, 0.1f, matches.size, inlierCount)
            }
            
            // Recover pose (R, t) from Essential Matrix
            val R = Mat()
            val t = Mat()
            Calib3d.recoverPose(
                essentialMatrix, 
                prevPoints, 
                currPoints,
                cameraMatrix,
                R, 
                t
            )
            
            // Extract translation (camera motion, so negate for world motion)
            val txVal = t.get(0, 0)
            val tyVal = t.get(2, 0)
            val tx = if (txVal != null && txVal.isNotEmpty()) -txVal[0].toFloat() * PIXEL_TO_METER else 0f
            val ty = if (tyVal != null && tyVal.isNotEmpty()) -tyVal[0].toFloat() * PIXEL_TO_METER else 0f
            
            // Extract rotation around Y axis (yaw) from rotation matrix
            val rotation = extractYawFromRotationMatrix(R)
            
            // Calculate confidence based on inlier ratio and match quality
            val inlierRatio = inlierCount.toFloat() / matches.size.toFloat()
            val avgDistance = matches.map { it.distance }.average().toFloat()
            val distanceConfidence = (1f - avgDistance / 64f).coerceIn(0f, 1f)
            val confidence = (inlierRatio * 0.7f + distanceConfidence * 0.3f).coerceIn(0f, 1f)
            
            Log.d(TAG, "Pose: tx=${"%.4f".format(tx)}, ty=${"%.4f".format(ty)}, rot=${"%.4f".format(rotation)}, " +
                    "inliers=$inlierCount/${matches.size}, conf=${"%.2f".format(confidence)}")
            
            // Clean up
            prevPoints.release()
            currPoints.release()
            essentialMatrix.release()
            mask.release()
            R.release()
            t.release()
            
            return MotionEstimate(tx, ty, rotation, confidence, matches.size, inlierCount)
            
        } catch (e: Exception) {
            Log.e(TAG, "Pose estimation error", e)
            return MotionEstimate(0f, 0f, 0f, 0f, matches.size, 0)
        }
    }
    
    /**
     * Extract yaw (rotation around vertical axis) from 3x3 rotation matrix.
     */
    private fun extractYawFromRotationMatrix(R: Mat): Float {
        // For a rotation matrix, yaw can be extracted as:
        // yaw = atan2(R[1,0], R[0,0]) for Z-Y-X convention
        return try {
            val r00 = R.get(0, 0)
            val r10 = R.get(1, 0)
            if (r00 != null && r10 != null && r00.isNotEmpty() && r10.isNotEmpty()) {
                atan2(r10[0], r00[0]).toFloat()
            } else {
                0f
            }
        } catch (e: Exception) {
            0f
        }
    }
    
    /**
     * Update global pose with motion estimate.
     */
    private fun updatePose(motion: MotionEstimate) {
        // Transform motion to world coordinates using current heading
        val worldDx = motion.deltaX * cos(currentPose.heading) - motion.deltaY * sin(currentPose.heading)
        val worldDy = motion.deltaX * sin(currentPose.heading) + motion.deltaY * cos(currentPose.heading)
        
        currentPose.x += worldDx
        currentPose.y += worldDy
        currentPose.heading += motion.rotation
        
        // Normalize heading to [0, 2PI]
        while (currentPose.heading < 0) currentPose.heading += (2 * PI).toFloat()
        while (currentPose.heading >= 2 * PI) currentPose.heading -= (2 * PI).toFloat()
    }
    
    // =====================
    // Public API (compatible with existing VisualOdometry)
    // =====================
    
    /**
     * Get detected features for visualization.
     */
    fun getCurrentFeatures(): List<FeaturePoint> {
        return prevKeypointList.map { kp ->
            // Classify as obstacle based on response (strength) and position
            val isObstacle = kp.response > 20 || 
                    (kp.x > CX * 0.5 && kp.x < CX * 1.5 && kp.y > CY * 0.5)
            FeaturePoint(
                x = kp.x,
                y = kp.y,
                score = kp.response,
                type = if (isObstacle) FeatureType.OBSTACLE else FeatureType.ENVIRONMENT
            )
        }
    }
    
    /**
     * Get current pose.
     */
    fun getCurrentPose(): Pose = currentPose.copy()
    
    /**
     * Get landmark count.
     */
    fun getLandmarkCount(): Int = landmarks.size
    
    /**
     * Reset visual odometry state.
     */
    fun reset() {
        prevDescriptors?.release()
        prevKeypointsMat?.release()
        prevDescriptors = null
        prevKeypointsMat = null
        prevKeypointList = emptyList()
        currentPose.x = 0f
        currentPose.y = 0f
        currentPose.heading = 0f
        landmarks.clear()
        landmarkIdCounter = 0
    }
    
    /**
     * Release all resources.
     */
    fun release() {
        reset()
        featureDetector.release()
    }
}
