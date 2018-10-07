package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;


public class Navx {
	private double NavxAngle = 0;
	private double originalAngle;
	private AHRS imu;

	public Navx(double angle, AHRS navx) {
		imu = navx;
		originalAngle = angle;
	}
	public double currentAngle() {
		return originalAngle -imu.getAngle();	
	}
	public boolean isMoving() {
		return imu.isMoving();
	}
	public boolean isOn(){
		return imu.isConnected();
	}
	public boolean isCalibrated(){
		return imu.isMagnetometerCalibrated();
	}
	public boolean isAutoCalibrating(){
		return imu.isCalibrating();
	}
	public boolean isInerference(){
		return imu.isMagneticDisturbance();
	}
	public void softReset(double angle){
		originalAngle = angle;

	}
	
	

}
