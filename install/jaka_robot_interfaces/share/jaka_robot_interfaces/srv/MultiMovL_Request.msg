# MultiMovL.srv

int32 robot_id
MoveMode left_move_mode
MoveMode right_move_mode
bool is_block
CartesianPose end_pos_left
CartesianPose end_pos_right
float64[] vel    # 长度为2
float64[] acc    # 长度为2
