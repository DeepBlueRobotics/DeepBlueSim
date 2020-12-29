package code.lib.sim;

import com.cyberbotics.webots.controller.PositionSensor;

import org.team199.wpiws.ScopedObject;
import org.team199.wpiws.UniqueArrayList;
import org.team199.wpiws.devices.EncoderSim;

public class MockedEncoder implements Runnable {
    private String name;
    private String wpiLibId;
    private EncoderSim encoder;
    private PositionSensor webotsEncoder;
    private int countsPerRevolution = 256;
    private int channelA = -1, channelB = -1;

    public MockedEncoder(UniqueArrayList<ScopedObject<?>> callbacks, EncoderSim sim, String wpiLibId) {
        encoder = sim;
        this.wpiLibId = wpiLibId;
        callbacks.add(sim.registerChannelACallback( (id, channel) -> {
            setChannelA(channel);
        }, true));
        callbacks.add(sim.registerChannelBCallback( (id, channel) -> {
            setChannelB(channel);
        }, true));
    }

    private void setChannelA(int channel) {
        channelA = channel;
        tryToConnectWebotsPositionSensor();
    }

    private void setChannelB(int channel) {
        channelB = channel;
        tryToConnectWebotsPositionSensor();
    }

    private void tryToConnectWebotsPositionSensor() {
        if (channelA < 0 || channelB < 0)
            return;
        String newName = "Encoder[" + channelA + "," + channelB + "]";
        if (webotsEncoder != null && !newName.equals(name)) {
            System.out.println("WARNING: Ignoring attempt to change PositionSensor of id " 
                + wpiLibId + " from " + name + " to " + newName);
            return;
        }

        // TODO: Find the position sensor whose name *starts* with the given
        // name and use the remained of the name to determine the countsPerRevolutions
        // to use.
        // For now, we just assume 256 countsPerRevolution
        webotsEncoder = Simulation.getRobot().getPositionSensor(newName);
        if(webotsEncoder != null) {
            name = newName;
            webotsEncoder.enable(BaseSimConfig.getSensorTimestep());
            Simulation.registerPeriodicMethod(this);
        }
    }

    private int prevCount = 0;
    private double timeCountChangedSecs = 0.0;
    private double timeCountCheckedSecs = 0.0;

    @Override
    public void run() {
        // Get the position of the Webots encoders and set the position of the WPIlib encoders 
        // getValue() returns radians
        double revolutions = (webotsEncoder.getValue()) / (2*Math.PI);
        int count = (int) Math.floor(revolutions * countsPerRevolution);
        encoder.setCount(count);

        // Compute the period of time since the previous tick. This is a bit more complicated than it would
        // seem at first glance because we need to handle both the case where multiple ticks have 
        // occured since we last checked, the case where no ticks have occurred across multiple
        // checks, and the case where only one tick has occured since we last checked.
        // For simplicity, we assume that if any number of ticks have occured, then the most recent one
        // happened now.
        double curTimeSecs = Simulation.getRobot().getTime();
        // If no ticks have happened since we last checked, then we know when the the previous tick happened.
        double prevTickTimeSecs = timeCountChangedSecs;
        // ... but if any ticks have happened since we last checked, then compute the time of the previous
        // tick assuming that the ticks were evenly spaced in time.
        if (count != prevCount) {
            prevTickTimeSecs = curTimeSecs - (curTimeSecs - timeCountCheckedSecs) / (count - prevCount);
        }
        double periodSecs = curTimeSecs - prevTickTimeSecs;
        encoder.setPeriod(periodSecs/1000.0); // WPILib expects ms.

        // Keep track of when we last checked the count and what it was.
        timeCountCheckedSecs = curTimeSecs;
        prevCount = count;

        // Keep track of when the count actually changed
        if (count != prevCount) {
            timeCountChangedSecs = curTimeSecs;
        }
    }
}