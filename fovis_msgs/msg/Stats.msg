# state of fovis
# same as fovis::stats_t lcm message
Header header

int32 num_matches
int32 num_inliers
int32 num_detected_keypoints
int32 num_reprojection_failures

# keypoints after pruning
int32 num_keypoints

float64 mean_reprojection_error
float64 fast_threshold

uint8 STATUS_CODE_NO_DATA = 0
uint8 STATUS_CODE_SUCCESS = 1
uint8 STATUS_CODE_INSUFFICIENT_INLIERS = 2
uint8 STATUS_CODE_OPTIMIZATION_FAILURE = 3
uint8 STATUS_CODE_REPROJECTION_ERROR = 4

uint8 status
