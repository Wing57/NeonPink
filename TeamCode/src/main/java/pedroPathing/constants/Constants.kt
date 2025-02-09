package pedroPathing.constants

object Constants {

    object Drive {

    }

    object Lift {
        const val SUBMERSIBLE = 60000.0
        const val AUTO = 39000.0
        const val CLIP = 32000

        const val TOP_BUCKET = 86000.0
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

        const val ARM_SEARCH = 0.6
        const val ARM_INTAKE = 0.4
        const val ARM_BUCKET = 0.5
        const val ARM_SPECIMEN = 0.8
        const val ARM_CLIP = 0.08 //.17

        const val PITCH_SEARCH = .3
        const val PITCH_GRAB = 0.24
        const val PITCH_BUCKET = 0.8
        const val PITCH_SPECIMEN = 0.68
        const val PITCH_AUTO_INIT = 0.1

        const val TWIST_ABNORMAL = 0.72
        const val TWIST_NORMAL = 0.16

    }
}