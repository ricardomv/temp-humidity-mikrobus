
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

#ifndef SENSORS_H
#define	SENSORS_H

float hih5030_get_sensor_rh(float voltage, float supply);

float hih5030_get_true_rh(float sensor_rh, float temperature);

float ad592_get_temp_degree_c(float voltage);

#endif	/* SENSORS_H */

