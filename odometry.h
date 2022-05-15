/*
 * odometry.h
 * This library allows to compute the position and angle of the ePuck
 * It contains a thread and functions
 * INPUT: motor's encoder steps values
 * OUTPUT: x and y position of the robot. Angle of the robot. All wrt its starting position
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

void get_position(Position* robot);
wrld_pos get_xPos(void);
wrld_pos get_yPos(void);
wrld_pos get_angle(void);
void odometry_start(void);

#endif /* ODOMETRY_H */
