import numpy as np


class IntentPredictCalc:
    def __init__(self):
        # Default positions and directions 
        self.hand_position = np.array([0, 0, 0], dtype=np.float32)  # in hand camera frame
        self.hand_direction = np.array([0, 0, 1], dtype=np.float32) # in hand camera frame
        self.gaze_direction = np.array([0, 0, 1], dtype=np.float32) # in head camera frame
        self.gaze_position = np.array([477, 233, 0], dtype=np.float32) # in head camera frame

    def find_grasp_obj(self, centroids, transformed_centroids, labels):
        """
        Predict the object the user intends to grasp
        Args:
            centroids: Object centroids in hand camera frame
            transformed_centroids: Object centroids transformed to head camera frame
            labels: Object IDs
            
        Returns:
            Index of the predicted grasp object
        """
        # find the closest object to the hand pointing vector
        hand_line_distances = self.find_closest_object_3d_line_optimized(centroids, self.hand_position, self.hand_direction)

        # find closest object to the hand in 3D space
        hand_3D_distances = self.find_closest_object_3D_depth(centroids, self.hand_position)

        # find closest object to the head gaze vector
        head_gaze_distances = self.find_closest_object_3d_line_optimized(transformed_centroids, self.gaze_position, self.gaze_direction)

        # combine the results and weight them
        closest_idx_final = self.weight_results(hand_line_distances, hand_3D_distances, head_gaze_distances)
        return closest_idx_final

    def find_closest_object_3d_line_optimized(self, objects_3d, head_hand_position, pointing_direction):
        """
        Find object closest to the pointing line in 3D space - optimized for NumPy arrays
        
        Args:
            objects_3d: NumPy array of shape (n, 3) with dtype=np.float32 containing object centroids
            hand_position: NumPy array of shape (3,) with hand camera position
            pointing_direction: NumPy array of shape (3,) with direction vector
            
        Returns:
            closest_idx: Index of closest object
            min_distance: Minimum perpendicular distance
            distances: Array of distances for all objects
        """
        # Ensure inputs are correct NumPy arrays
        head_hand_position = np.asarray(head_hand_position, dtype=np.float32)
        
        # Normalize direction vector (in-place if possible)
        direction = np.asarray(pointing_direction, dtype=np.float32)
        direction_norm = np.linalg.norm(direction)
        if direction_norm > 0:
            direction = direction / direction_norm
        
        # Vector from hand/head to all objects (n, 3)
        vectors = objects_3d - head_hand_position
        
        # Scalar projections of all vectors onto direction
        proj_lengths = np.dot(vectors, direction)  # shape: (n,)
        
        # Initialize distances array with infinite values
        distances = np.full(len(objects_3d), np.inf, dtype=np.float32)
        
        # Find objects in front of the hand/head camera (positive projection)
        front_mask = proj_lengths > 0
        
        # For objects in front, calculate perpendicular distances
        # Reshape for broadcasting
        proj_lengths_front = proj_lengths[front_mask].reshape(-1, 1)
        direction_reshaped = direction.reshape(1, 3)
        
        # Calculate closest points on ray for all objects at once
        closest_points = head_hand_position + proj_lengths_front * direction_reshaped
        
        # Calculate perpendicular distances (vectorized)
        perp_distances = np.linalg.norm(objects_3d[front_mask] - closest_points, axis=1)
        
        # Store distances for objects in front
        distances[front_mask] = perp_distances
            
        return distances
    

    def find_closest_object_3D_depth(self, centroids, object_pos):
        """
        Find the closest object to the hand based on 3D euclidean distance - optimized for NumPy arrays
        
        Args:
            centroids: NumPy array of shape (n, 3) containing object centroids
            object_pos: NumPy array of shape (3,) containing the reference position
        
        Returns:
            closest_idx: Index of closest object
        """
        # Ensure inputs are NumPy arrays
        centroids_array = np.asarray(centroids, dtype=np.float32)
        object_pos_array = np.asarray(object_pos, dtype=np.float32)
        
        # Calculate all distances at once using broadcasting
        # This computes the Euclidean distance between object_pos and each centroid
        distances = np.linalg.norm(centroids_array - object_pos_array, axis=1)
        
        return distances 

    def distance_to_score(self, distances, max_distance=1.0, score_range=(0, 100)):
        """
        Convert distances to scores - closer objects get higher scores
        
        Args:
            distances: NumPy array of distances to objects in mm
            max_distance: Distance beyond which score is minimum (meters)
            score_range: Tuple of (min_score, max_score)
        
        Returns:
            scores: NumPy array of scores for each object
        """
        # Ensure distances is a NumPy array
        distances = np.asarray(distances)
        
        # Clip distances to max_distance to avoid negative scores
        clipped_distances = np.clip(distances, 0, max_distance)
        
        # Convert distances to scores (inverse relationship: closer = higher score)
        # 1 - (d/max_d) gives a value between 0 and 1, where 0 distance = 1, max distance = 0
        normalized_scores = 1.0 - (clipped_distances / max_distance)
        
        # Scale to requested score range
        min_score, max_score = score_range
        scores = min_score + normalized_scores * (max_score - min_score)
        
        return scores
    

    def weight_results(self, hand_line_distances, hand_depth_distances, gaze_line_distances):
        """
        Combine the results from the three methods and weight them
        with proper normalization and score conversion
        
        Args:
            hand_line_distances: Distances from hand pointing line to objects
            hand_depth_distances: 3D distances from hand to objects
            gaze_line_distances: Distances from gaze line to objects
            
        Returns:
            closest_idx: Index of the object with the highest combined score
        """
        # 1. Convert each distance metric to a normalized score (higher = better)
        # Use appropriate max_distance values based on your environment
        hand_line_scores = self.distance_to_score(hand_line_distances, 
                                                max_distance=0.5,  # 50cm max perpendicular distance for pointing
                                                score_range=(0, 100))
        
        hand_depth_scores = self.distance_to_score(hand_depth_distances, 
                                                max_distance=1.0,  # 1m max 3D distance 
                                                score_range=(0, 100))
        
        gaze_line_scores = self.distance_to_score(gaze_line_distances, 
                                                max_distance=0.5,  # 50cm max perpendicular distance for gaze
                                                score_range=(0, 100))
        
        # 2. Dynamically adjust weights based on hand proximity
        min_hand_dist = np.min(hand_depth_distances) if len(hand_depth_distances) > 0 else float('inf')
        
        if min_hand_dist < 0.15:  # Close to object (15cm)
            # Hand position dominates when close to objects
            weight_line = 0.2   # Reduce importance of pointing line
            weight_hand = 0.7   # Increase importance of hand proximity
            weight_gaze = 0.1   # Reduce importance of gaze
        elif min_hand_dist < 0.4:  # Medium distance (40cm)
            # Balanced weights at medium distance
            weight_line = 0.4   # Moderate pointing importance
            weight_hand = 0.3   # Moderate hand proximity importance
            weight_gaze = 0.3   # Moderate gaze importance
        else:  # Far from objects
            # Gaze and pointing direction dominate at a distance
            weight_line = 0.45  # High pointing importance
            weight_hand = 0.1   # Low hand proximity importance
            weight_gaze = 0.45  # High gaze importance
        
        # 3. Calculate combined scores using weights
        combined_scores = (weight_line * hand_line_scores + 
                        weight_hand * hand_depth_scores + 
                        weight_gaze * gaze_line_scores)
        
        # 4. Find index of object with highest score
        closest_idx = np.argmax(combined_scores)
        
        return closest_idx



