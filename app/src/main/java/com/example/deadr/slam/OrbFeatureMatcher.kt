package com.example.deadr.slam

import android.util.Log
import org.opencv.core.DMatch
import org.opencv.core.Mat
import org.opencv.core.MatOfDMatch
import org.opencv.core.MatOfKeyPoint
import org.opencv.features2d.BFMatcher
import org.opencv.features2d.DescriptorMatcher

/**
 * ORB Feature Matcher using OpenCV Brute-Force with Hamming distance.
 * Implements Lowe's ratio test for robust matching.
 */
class OrbFeatureMatcher {
    
    companion object {
        private const val TAG = "OrbFeatureMatcher"
        
        // Matching parameters
        private const val LOWE_RATIO = 0.75f          // Lowe's ratio test threshold
        private const val MAX_DISTANCE = 64f          // Maximum Hamming distance for valid match
        private const val MIN_MATCHES = 10            // Minimum matches required
    }
    
    // Brute-force matcher with Hamming distance (for binary descriptors)
    private var matcher: BFMatcher? = null
    
    /**
     * Matched feature pair between frames.
     */
    data class FeatureMatch(
        val queryIdx: Int,
        val trainIdx: Int,
        val queryX: Float,
        val queryY: Float,
        val trainX: Float,
        val trainY: Float,
        val distance: Float
    )
    
    /**
     * Result of feature matching.
     */
    data class MatchResult(
        val matches: List<FeatureMatch>,
        val inlierRatio: Float
    )
    
    init {
        try {
            // Use BRUTEFORCE_HAMMING for binary ORB descriptors
            matcher = BFMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING, false)
            Log.d(TAG, "BFMatcher created successfully")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to create BFMatcher", e)
        }
    }
    
    /**
     * Match features between two frames using ratio test.
     * 
     * @param prevDescriptors Descriptors from previous frame
     * @param prevKeypoints Keypoints from previous frame
     * @param currDescriptors Descriptors from current frame
     * @param currKeypoints Keypoints from current frame
     * @return MatchResult with filtered matches
     */
    fun match(
        prevDescriptors: Mat,
        prevKeypoints: MatOfKeyPoint,
        currDescriptors: Mat,
        currKeypoints: MatOfKeyPoint
    ): MatchResult? {
        val bfMatcher = matcher ?: return null
        
        if (prevDescriptors.empty() || currDescriptors.empty()) {
            Log.w(TAG, "Empty descriptors")
            return null
        }
        
        if (prevDescriptors.rows() < 2 || currDescriptors.rows() < 2) {
            Log.w(TAG, "Not enough descriptors for matching")
            return null
        }
        
        try {
            // KNN match with k=2 for ratio test
            val knnMatches = mutableListOf<MatOfDMatch>()
            bfMatcher.knnMatch(prevDescriptors, currDescriptors, knnMatches, 2)
            
            val prevKpList = prevKeypoints.toList()
            val currKpList = currKeypoints.toList()
            
            // Apply Lowe's ratio test
            val goodMatches = mutableListOf<FeatureMatch>()
            
            for (matchMat in knnMatches) {
                val matches = matchMat.toList()
                if (matches.size >= 2) {
                    val m = matches[0]
                    val n = matches[1]
                    
                    // Ratio test: best match should be significantly better than second best
                    if (m.distance < LOWE_RATIO * n.distance && m.distance < MAX_DISTANCE) {
                        if (m.queryIdx < prevKpList.size && m.trainIdx < currKpList.size) {
                            val prevKp = prevKpList[m.queryIdx]
                            val currKp = currKpList[m.trainIdx]
                            
                            goodMatches.add(FeatureMatch(
                                queryIdx = m.queryIdx,
                                trainIdx = m.trainIdx,
                                queryX = prevKp.pt.x.toFloat(),
                                queryY = prevKp.pt.y.toFloat(),
                                trainX = currKp.pt.x.toFloat(),
                                trainY = currKp.pt.y.toFloat(),
                                distance = m.distance
                            ))
                        }
                    }
                }
            }
            
            val inlierRatio = if (knnMatches.isNotEmpty()) {
                goodMatches.size.toFloat() / knnMatches.size.toFloat()
            } else 0f
            
            Log.d(TAG, "Matched ${goodMatches.size}/${knnMatches.size} features (ratio: ${"%.2f".format(inlierRatio)})")
            
            return MatchResult(goodMatches, inlierRatio)
            
        } catch (e: Exception) {
            Log.e(TAG, "Feature matching failed", e)
            return null
        }
    }
    
    /**
     * Simple match without ratio test (faster, less robust).
     */
    fun matchSimple(
        prevDescriptors: Mat,
        prevKeypoints: MatOfKeyPoint,
        currDescriptors: Mat,
        currKeypoints: MatOfKeyPoint
    ): MatchResult? {
        val bfMatcher = matcher ?: return null
        
        if (prevDescriptors.empty() || currDescriptors.empty()) {
            return null
        }
        
        try {
            val matches = MatOfDMatch()
            bfMatcher.match(prevDescriptors, currDescriptors, matches)
            
            val prevKpList = prevKeypoints.toList()
            val currKpList = currKeypoints.toList()
            val matchList = matches.toList()
            
            // Filter by distance threshold
            val goodMatches = matchList
                .filter { it.distance < MAX_DISTANCE }
                .filter { it.queryIdx < prevKpList.size && it.trainIdx < currKpList.size }
                .map { m ->
                    val prevKp = prevKpList[m.queryIdx]
                    val currKp = currKpList[m.trainIdx]
                    FeatureMatch(
                        queryIdx = m.queryIdx,
                        trainIdx = m.trainIdx,
                        queryX = prevKp.pt.x.toFloat(),
                        queryY = prevKp.pt.y.toFloat(),
                        trainX = currKp.pt.x.toFloat(),
                        trainY = currKp.pt.y.toFloat(),
                        distance = m.distance
                    )
                }
            
            val inlierRatio = if (matchList.isNotEmpty()) {
                goodMatches.size.toFloat() / matchList.size.toFloat()
            } else 0f
            
            return MatchResult(goodMatches, inlierRatio)
            
        } catch (e: Exception) {
            Log.e(TAG, "Simple matching failed", e)
            return null
        }
    }
}
