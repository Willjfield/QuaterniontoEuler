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
//var quat= [qx,qy,qz,qw]
var quat = {
	w:.9437,
	x:.2685,
	y:.1277,
	z:.1448
}
console.log(toEuler(quat))
