package org.carlmontrobotics.libdeepbluesim.internal;

/**
 * Constants that are shared between libdeepbluesim's WebotsSimulator DeepBlueSim's WebotsSupervisor
 * that allow them to communicate with each other over NetworkTables. SUBJECT TO CHANGE SO NOT FOR
 * USE BY OTHERS.
 */
public class NTConstants {
        /** The name of the topic that is used to communicate the desired Webots simulation mode. */
        public static final String SIM_MODE_TOPIC_NAME = "simMode";

        /** The value signifying that Webots should run in real-time mode. */
        public static final String SIM_MODE_REALTIME_VALUE = "Realtime";

        /** The value signifying that Webots should run in fast mode. */
        public static final String SIM_MODE_FAST_VALUE = "Fast";

        /**
         * The name of the topic that is used to communicate the current Webots simulation time (in
         * seconds).
         */
        public static final String SIM_TIME_SEC_TOPIC_NAME = "simTimeSec";

        /**
         * The name of the topic that is used to communicate the current robot code simulation time
         * (in seconds).
         */
        public static final String ROBOT_TIME_SEC_TOPIC_NAME = "robotTimeSec";

        /** The name of the topic that is used to communicate the status of Webots (re)loading. */
        public static final String RELOAD_STATUS_TOPIC_NAME = "reloadStatus";

        /** The value signifying that Webots has finished reloading. */
        public static final String RELOAD_STATUS_COMPLETED_VALUE = "Completed";

        /** The name of the topic that is used to tell Webots which world file to load. */
        public static final String RELOAD_REQUEST_TOPIC_NAME = "reloadRequest";

        /** The name of the table that contains the above topics. */
        public static final String COORDINATOR_TABLE_NAME =
                        "/DeepBlueSim/Coordinator";

        /**
         * The name of the table that has a subtable for each Webots node that information should be
         * reported for.
         */
        public static final String WATCHED_NODES_TABLE_NAME =
                        "/DeepBlueSim/WatchedNodes";

        /** The name of the topic that is used to report position information. */
        public static final String POSITION_TOPIC_NAME = "position";

        /** The name of the topic that is used to report rotation information. */
        public static final String ROTATION_TOPIC_NAME = "rotation";

        /** The name of the topic that is used to report velocity information. */
        public static final String VELOCITY_TOPIC_NAME = "velocity";
}
