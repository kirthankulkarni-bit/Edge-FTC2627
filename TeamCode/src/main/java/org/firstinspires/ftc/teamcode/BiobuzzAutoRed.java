package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BIOBUZZ auto, red alliance. All the logic is in BiobuzzAuto. */
@Autonomous(name = "BIOBUZZ Auto Red", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class BiobuzzAutoRed extends BiobuzzAuto {
    @Override
    protected boolean isRed() { return true; }
}
