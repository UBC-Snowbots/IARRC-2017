# import the necessary packages
import numpy as np
import imutils
import cv2

class Stitcher:
	def __init__(self):
		# determine if we are using OpenCV v3.X and initialize the
		# cached homography matrix
		self.isv3 = imutils.is_cv3()
		self.cachedH_l = None
		self.cachedH_r = None
		self.cachedH = None
		
	def stitch(self, images, direction=1, ratio=0.75, reprojThresh=4.0):
		# unpack the images
		if direction == 0:
			(imageB, imageA) = images
		else:
			(imageC, imageB, imageA) = images

		# if the cached homography matrix is None, then we need to
		# apply keypoint matching to construct it
		if self.cachedH_l is None and self.cachedH_r is None:
		#if self.cachedH is None:
			# detect keypoints and extract
			(kpsA, featuresA) = self.detectAndDescribe(imageA)
			(kpsB, featuresB) = self.detectAndDescribe(imageB)
			(kpsC, featuresC) = self.detectAndDescribe(imageC)

			# match features between the two images
			M_r = self.matchKeypoints(kpsA, kpsB,
				featuresA, featuresB, ratio, reprojThresh)
			
			M_l = self.matchKeypoints(kpsC, kpsB,
				featuresC, featuresB, ratio, reprojThresh)

			#Move the left transforamtion 400 cols to the right in order to 
			#stitch it to the middle one
			Offset = np.array([[1, 0,  400],
				  	   [0, 1,   0 ],
				  	   [0, 0,   1 ]])

			M_l_offset = M_l[1].dot(Offset) 

			# if the match is None, then there aren't enough matched
			# keypoints to create a panorama
			if M_r is None or M_l is None:
				return None

			# cache the homography matrix
			self.cachedH_l = M_l_offset
			self.cachedH_r = M_r[1]
			#self.cachedH_l_inv = M_l_inv[1]

			
		# apply a perspective transform to stitch the images together
		# using the cached homography matrix
		#.shape returns a tuple of number of rows, columns and channels
		#warp right imageA and left imageC to match the middle imageB
		result_l = cv2.warpPerspective(imageC, self.cachedH_l,
			(imageC.shape[1] + imageB.shape[1], imageC.shape[0]))
		result_r = cv2.warpPerspective(imageA, self.cachedH_r, 
			(imageA.shape[1] + imageB.shape[1], imageA.shape[0]))
		result = cv2.warpPerspective(imageA, self.cachedH_r,
			(imageA.shape[1] * 3, imageA.shape[0]))
	
		result[0:result_l.shape[0], 400: result_l.shape[1]*3] = result_r
		result[0:result_l.shape[0], 0:800] = result_l
		result[0:300, 400:800] = imageB

		# return the stitched image
		return result

	def detectAndDescribe(self, image):
		# convert the image to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# check to see if we are using OpenCV 3.X
		if self.isv3:
			# detect and extract features from the image
			descriptor = cv2.xfeatures2d.SIFT_create()
			(kps, features) = descriptor.detectAndCompute(image, None)

		# otherwise, we are using OpenCV 2.4.X
		else:
			# detect keypoints in the image
			detector = cv2.FeatureDetector_create("SIFT")
			kps = detector.detect(gray)

			# extract features from the image
			extractor = cv2.DescriptorExtractor_create("SIFT")
			(kps, features) = extractor.compute(gray, kps)

		# convert the keypoints from KeyPoint objects to NumPy
		# arrays
		kps = np.float32([kp.pt for kp in kps])

		# return a tuple of keypoints and features
		return (kps, features)

	def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
		ratio, reprojThresh):
		# compute the raw matches and initialize the list of actual
		# matches
		matcher = cv2.DescriptorMatcher_create("BruteForce")
		rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
		matches = []

		# loop over the raw matches
		for m in rawMatches:
			# ensure the distance is within a certain ratio of each
			# other (i.e. Lowe's ratio test)
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				matches.append((m[0].trainIdx, m[0].queryIdx))

		# computing a homography requires at least 4 matches
		if len(matches) > 4:
			# construct the two sets of points
			ptsA = np.float32([kpsA[i] for (_, i) in matches])
			ptsB = np.float32([kpsB[i] for (i, _) in matches])

			# compute the homography between the two sets of points
			(H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
				reprojThresh)

			# return the matches along with the homograpy matrix
			# and status of each matched point
			return (matches, H, status)

		# otherwise, no homograpy could be computed
		return None
