package pedroPathing.constants

object Constants {
    //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

    object Drive {

    }

    object Lift {
        const val SUBMERSIBLE = 60000.0
        const val AUTO = 39000.0
        const val CLIP = 32000

        const val TOP_BUCKET = 87000.0
        const val ZERO = 10000.0;
        const val AXEL_ZERO = 10000.0
        const val AUTO_ZERO = 0
        const val SPECIMEN = 2500.0

        const val DOWN = 2100.0
        const val UP = 0
        const val AUTO_PIVOT = 1200

    }

    object Servos {
        const val OPEN = 0.48
        const val CLOSE = 0.3

        const val ARM_SEARCH = 0.45 //good
        const val ARM_INTAKE = 0.3  //good
        const val ARM_BUCKET = 0.43
        const val ARM_SPECIMEN = 0.83 // good
        const val ARM_CLIP = 0.0//good
        const val ARM_STORE = 0.53

        const val PITCH_SEARCH = 0.2
        const val PITCH_GRAB = 0.1 // good
        const val PITCH_BUCKET = 1
        const val PITCH_SPECIMEN = 0.7 //
        const val PITCH_AUTO_INIT = 0.95

        const val TWIST_ABNORMAL = 0.95 //good
        const val TWIST_NORMAL = 0.4 //good

        const val SWEEP_UP = 0.6
        const val SWEEP_DOWN = 1

    }
}