package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.drive.Constants;

public class commands {
    // Method to check the delivery conditions and set the bucket position
    public void scoreSequence(Constants constants, int dump) {
        // Ensure constants and dump are valid to avoid potential NPEs
        if (constants != null && constants.delivery &&
                Math.abs(constants.slide.getCurrentPosition() - constants.currentBasket) < 1000) {

            // Set the bucket position if the conditions are met
            constants.bucket.setPosition(dump);
        }
    }
}
