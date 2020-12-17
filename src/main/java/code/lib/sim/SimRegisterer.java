package code.lib.sim;

import java.util.function.Function;

import org.team199.wpiws.EncoderSim;
import org.team199.wpiws.PWMSim;
import org.team199.wpiws.ScopedObject;
import org.team199.wpiws.SimDeviceSim;
import org.team199.wpiws.UniqueArrayList;
import org.team199.wpiws.interfaces.BiLongCallback;
import org.team199.wpiws.interfaces.BooleanCallback;
import org.team199.wpiws.interfaces.SimDeviceCallback;

import code.lib.Future;

// Performs automatic registration of callbacks detecting both the initalization of new devices as well as data callbacks for devices such as Motors, Gyros, etc.
// This allows us to automatically link these devices to Webots, reducing the amount of code we would have to change from a standard robot project
public class SimRegisterer {
    
    private static final SimDeviceCallback MISC_DEVICE_CALLBACK = SimRegisterer::callback;
    private static final UniqueArrayList<ScopedObject<?>> CALLBACKS = new UniqueArrayList<>();

    static {
        // Register Initalized Callbacks for Misc Devices
        SimDeviceSim.registerDeviceCreatedCallback("", MISC_DEVICE_CALLBACK, true);
        // Register Initalized Callbacks for PWM Devices
        registerCallback(BooleanCallback.class, callback -> PWMSim.registerStaticInitializedCallback((name, isInitialized) -> {
            if(isInitialized) {
                callback("PWM", name, 0);
            }
        }, true));
        // Register Initalized Callbacks for Encoder Devices
        registerCallback(BooleanCallback.class, callbackU -> EncoderSim.registerStaticInitializedCallback((name, isInitialized) -> {
            if(isInitialized) {
                //When an Encoder is initalized, wait for it's channels to become available
                registerCallback(BiLongCallback.class, callback -> new EncoderSim(name).registerChannelInitializedCallback((nameU, channelAL, channelB) -> {
                    int channelA = (int)channelAL;
                    PWMSim pwmSim = new PWMSim(channelA/2);
                    //Then wait for the associated PWM motor to be initalized
                    registerCallback(BooleanCallback.class, callback2 -> pwmSim.registerInitializedCallback((nameU2, isInitialized2) -> {
                        if(isInitialized2) {
                            //Then sync the values of that motor's encoder to the sim device
                            new MockedSparkEncoder("PWM_" + (channelA/2), channelA/2);
                        }
                        callback2.close();
                    }, true));
                    callback.close();
                }, true));
            }
        }, true));
    }

    // Initalize SimRegisterer. This method exists to ensure that the static block is called
    public static void init() {}

    // Register callbacks for known devices on Non-Can ports
    @SuppressWarnings("all")
    private static <T> void registerCallback(Class<T> callbackType, Function<ScopedObject<T>, ScopedObject<T>> callbackRegisterFunc) {
        Future<ScopedObject<T>> callback = new Future(ScopedObject.class);
        callback.set(callbackRegisterFunc.apply(callback.get()));
    }

    // Callback methods which place new devices in a processing queue which is processed every robot period
    // The WPILib callbacks are notified as part of the device creation. This process ensures that the devices complete their setup process
    // This is especially important for SimDevice's because their initalized callbacks can be notified before their values have been created
    // Queuing also ensures that callbacks (which are usually executed asycronously) are processed syncronously with the rest of the robot code

    // Callback for when a Miscellaneous Device is registered
    private static void callback(String deviceName) {
        if(deviceName.startsWith("Talon") || deviceName.startsWith("Victor")) {
            // If a new Talon or Victor has been initalized, attempt to link it to a Webots Motor
            // Create a WebotsMotorForwarder for this motor
            final WebotsMotorForwarder fwdr = new WebotsMotorForwarder(Simulation.getRobot(), deviceName);
            // Register a callback for when the Motor Output changes
            CALLBACKS.add(new SimDeviceSim(deviceName).registerValueChangedCallback("Motor Output",
                // Call the callback function
                fwdr,
                // Initalize with current speed
                true));
        }
        if(deviceName.startsWith("navX")) {
            // If a navX is registered, try to link its SimDevice to the Webots robot
            MockGyro.linkGyro();
        }
    }

    // Callback for when a known device type is registered on a Non-Can port
    private static void callback(String type, String port, int storePos) {
        if(type.equals("PWM")) {
            // If a new PWM device has been initalized, attempt to link it to a Webots Motor
            // Register a speed callback on this device
            CALLBACKS.add(new PWMSim(port).registerSpeedCallback(
                // Call a motor forwarder for a callback
                new WebotsMotorForwarder(Simulation.getRobot(), "PWM_" + port),
                // Initalize with current speed
                true));
        }
    }

}