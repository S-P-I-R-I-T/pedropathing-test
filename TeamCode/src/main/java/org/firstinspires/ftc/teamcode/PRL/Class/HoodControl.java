package org.firstinspires.ftc.teamcode.PRL.Class;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;

@Configurable
public class HoodControl {
    private final ActionManaging action;
    private final Follower follower;

    public static double Hood_Min = 0.4;
    public static double Hood_Max = 0.8;

    public static double Y_Near = 8;
    public static double Y_Far = 119;
    public static double Hood_Y_Near = 1;
    public static double Hood_Y_Far = 0;

    public HoodControl(ActionManaging action, Follower follower){
        this.action = action;
        this.follower = follower;
    }

    public void update(){
        double y = follower.getPose().getY();

        double hood = Hood_Y_Near + (Hood_Y_Far - Hood_Y_Near) * (y - Y_Near) / (Y_Far - Y_Near);

        hood = Math.max(Hood_Min, Math.min(Hood_Max, hood));

        action.Hood_Set(hood);
    }

}
