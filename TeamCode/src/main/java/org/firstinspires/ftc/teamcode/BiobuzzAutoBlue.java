package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BIOBUZZ auto, blue alliance. Same route as red, rotated 180 degrees. Logic is in BiobuzzAuto. */
@Autonomous(name = "BIOBUZZ Auto Blue", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class BiobuzzAutoBlue extends BiobuzzAuto {
    @Override
    protected boolean isRed() { return false; }
}
