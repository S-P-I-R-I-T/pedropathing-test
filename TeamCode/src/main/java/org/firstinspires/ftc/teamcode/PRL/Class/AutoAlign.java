package org.firstinspires.ftc.teamcode.PRL.Class;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class AutoAlign {
    public static double POWER_SCALE = 0.8; // 파워 게인 (작을수록 느리게, 클수록 빠르게)
    public static double MIN_POWER = 0.1;   // 마찰을 이기는 최소 파워. 오차가 DEADBAND보다 크면 이 이상은 가함
    public static double DEADBAND = 3;      // 오차가 이 각도(도) 이하면 정지

    ActionManaging action;
    Follower follower;
    public AutoAlign(ActionManaging action, Follower follower){
        this.action = action;
        this.follower = follower;
    }

    public double getTheta(){
        double absForwardPodValue = Math.abs(103-follower.getPose().getX());
        double absStrafePodValue = Math.abs(59-follower.getPose().getY());
        return Math.toDegrees(Math.atan(absForwardPodValue/absStrafePodValue));
    }

    public void stop(){
        action.Turret_SetPower(0);
    }

    public double autoAlign(boolean isBlue){
        double robotHeading = Math.toDegrees(follower.getPose().getHeading());
        double targetAngle;
        if (!isBlue){
            targetAngle = -(90-getTheta()) + robotHeading;
        }
        else {
            targetAngle = -(90-getTheta()) - robotHeading;
        }

        // 현재 터렛 각도를 읽어 오차를 -180~180으로 정규화 → 항상 가까운 쪽(최대 180도)으로 회전
        double current = action.Turret_CurrentAngle();
        double error = normalizeAngle(targetAngle - current);

        if (Math.abs(error) < DEADBAND) {
            action.Turret_SetPower(0);
        } else {
            double power = POWER_SCALE * (error/360.0);
            power = Math.copySign(Math.max(Math.abs(power), MIN_POWER), power);
            action.Turret_SetPower(power);
        }
        return error;
    }

    private double normalizeAngle(double deg){
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
