var fs = require('fs')

function toEuler(q1) {
    var sqw = q1.w*q1.w;
    var sqx = q1.x*q1.x;
    var sqy = q1.y*q1.y;
    var sqz = q1.z*q1.z;

	var unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
	var test = q1.x*q1.y + q1.z*q1.w;
	if (test > 0.499*unit) { // singularity at north pole
		heading = 2 * Math.atan2(q1.x,q1.w);
		attitude = Math.PI/2;
		bank = 0;
		return;
	}
	if (test < -0.499*unit) { // singularity at south pole
		heading = -2 * Math.atan2(q1.x,q1.w);
		attitude = -Math.PI/2;
		bank = 0;
		return;
	}
        heading = Math.atan2(2*q1.y*q1.w-2*q1.x*q1.z , sqx - sqy - sqz + sqw);
	attitude = Math.asin(2*test/unit);
	bank = Math.atan2(2*q1.x*q1.w-2*q1.y*q1.z , -sqx + sqy - sqz + sqw)
	
	heading*=(180/Math.PI)
	attitude*=(180/Math.PI)
	bank*=(180/Math.PI)
	return [heading,attitude,bank]
}
var qw = 0.9437  
var qx = 0.2685  
var qy = 0.1277  
var qz = 0.1448

var quat = {
	w:.9437,
	x:.2685,
	y:.1277,
	z:.1448
}

var parseText = function(txt){
			txt = txt.split('\r');
			for(e in txt){
				var ne=txt[e].split(',');
				ne.splice(0,7)
				ne.splice(9)
				for(n in ne){
					ne[n] = parseFloat(ne[n])
					}
				txt[e]=ne;
				}
				return txt;
			}
fs.readFile('log_1.csv', 'utf8',function(err,data){
	//console.log(parseText(data))
	var _data = parseText(data)
	for(var i = 15; i<_data.length;i++){
	var gx = _data[i][6]
	var gy = _data[i][7]
	var gz = _data[i][8]
	var ax = _data[i][0]
	var ay = _data[i][1]
	var az = _data[i][2]
	console.log(toEuler(mIMU(gx,gy,gz,ax,ay,az)))

	}

})

//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date Author Notes
// 29/09/2011 SOH Madgwick Initial release
// 02/10/2011 SOH Madgwick Optimised for reduced CPU load
// 19/02/2012 SOH Madgwick Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Definitions
var sampleFreq = 32.0; // sample frequency in Hz
var betaDef = 1; // 2 * proportional gain
//---------------------------------------------------------------------------------------------------
// Variable definitions
var beta = betaDef; // 2 * proportional gain (Kp)
var q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0; // quaternion of sensor frame relative to auxiliary frame
console.log("Madgwick here");
//====================================================================================================
// Functions
//---------------------------------------------------------------------------------------------------
// IMU algorithm update
function mIMU(gx, gy, gz, ax, ay, az) {
var recipNorm;
var s0, s1, s2, s3;
var qDot1, qDot2, qDot3, qDot4;
var V_2q0, V_2q1, V_2q2, V_2q3, V_4q0, V_4q1, V_4q2, V_8q1, V_8q2, q0q0, q1q1, q2q2, q3q3;
// Rate of change of quaternion from gyroscope
qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
if (!((ax === 0.0) && (ay === 0.0) && (az === 0.0))) {
// Normalise accelerometer measurement
recipNorm = Math.pow(ax * ax + ay * ay + az * az, -0.5);
ax *= recipNorm;
ay *= recipNorm;
az *= recipNorm;
// Auxiliary variables to avoid repeated arithmetic
V_2q0 = 2.0 * q0;
V_2q1 = 2.0 * q1;
V_2q2 = 2.0 * q2;
V_2q3 = 2.0 * q3;
V_4q0 = 4.0 * q0;
V_4q1 = 4.0 * q1;
V_4q2 = 4.0 * q2;
V_8q1 = 8.0 * q1;
V_8q2 = 8.0 * q2;
q0q0 = q0 * q0;
q1q1 = q1 * q1;
q2q2 = q2 * q2;
q3q3 = q3 * q3;
// Gradient decent algorithm corrective step
s0 = V_4q0 * q2q2 + V_2q2 * ax + V_4q0 * q1q1 - V_2q1 * ay;
s1 = V_4q1 * q3q3 - V_2q3 * ax + 4.0 * q0q0 * q1 - V_2q0 * ay - V_4q1 + V_8q1 * q1q1 + V_8q1 * q2q2 + V_4q1 * az;
s2 = 4.0 * q0q0 * q2 + V_2q0 * ax + V_4q2 * q3q3 - V_2q3 * ay - V_4q2 + V_8q2 * q1q1 + V_8q2 * q2q2 + V_4q2 * az;
s3 = 4.0 * q1q1 * q3 - V_2q1 * ax + 4.0 * q2q2 * q3 - V_2q2 * ay;
recipNorm = Math.pow(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3, -0.5); // normalise step magnitude
s0 *= recipNorm;
s1 *= recipNorm;
s2 *= recipNorm;
s3 *= recipNorm;
// Apply feedback step
qDot1 -= beta * s0;
qDot2 -= beta * s1;
qDot3 -= beta * s2;
qDot4 -= beta * s3;
}
// Integrate rate of change of quaternion to yield quaternion
q0 += qDot1 * (1.0 / sampleFreq);
q1 += qDot2 * (1.0 / sampleFreq);
q2 += qDot3 * (1.0 / sampleFreq);
q3 += qDot4 * (1.0 / sampleFreq);
// Normalise quaternion
recipNorm = Math.pow(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3, -0.5);
q0 *= recipNorm;
q1 *= recipNorm;
q2 *= recipNorm;
q3 *= recipNorm;

return {
	x:q0,
	y:q1,
	z:q2,
	w:q3
	}
}
