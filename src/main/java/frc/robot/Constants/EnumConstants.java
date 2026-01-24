package frc.robot.constants;

public class EnumConstants {
    public enum AlgaeMode {
        PROCESSOR, REEFLOW, REEFHIGH, NET, GROUND;
    }

    public enum CoralMode {
        TROUGH, LOW, MID, HIGH;
    }

    public static enum AlignPoint {
        A, B, C, D, E, F, G, H, I, J, K, L, OTHERA, OTHERB, OTHERC, OTHERD, OTHERE, OTHERF,
        HUMANPLAYER1, HUMANPLAYER2,
        CLIMB1, CLIMB2, CLIMB3,
        CORALLEFT, CORALRIGHT,
        ALGAE, LOLLIPOP1, LOLLIPOP2, LOLLIPOP3, BARGE;
    };

    public enum IntakeMode {
        GROUND, HUMAN, REVERSE, HOME, HANDOFF, CLIMB;
    }

    public enum ScoringMode {
        ALGAE, CORAL, CLIMB, HUMANPLAYER;
    }
}
