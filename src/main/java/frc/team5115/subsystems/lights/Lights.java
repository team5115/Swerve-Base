package frc.team5115.subsystems.lights;
import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;

public class Lights extends SubsystemBase {
    private enum AnimationState { Idle, ControllingNote, Aligning, InRange }

    private final int period = 2;
    private final double tailLength = 5;
    private final double decay = 1d / tailLength;
    private final int minPower = 10;
    private final int maxPower = 250;
    
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    public final int ledCount;

    private int timer_idle;
    private int counter_idle;
    private int direction_idle;

    private int timer_align;
    private int position_align;

    private DigitalInput reflectiveSensor;

    public Lights(int port, int ledCount){
        this.ledCount = ledCount;
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(ledCount);
        reflectiveSensor = new DigitalInput(Constants.SHOOTER_SENSOR_ID);

    }

    public void start(){
        leds.start();
    }

    public void stop(){
        leds.stop();
    }

    public void iterateAllLeds(Function<Integer, Integer[]> function) { //only use this function when turning LEDS on 
        int ledsOnCount = 0;
        for (int i = 0; i < ledCount; i++) {
            Integer[] color = function.apply(i);
            if (color[0] > 0 || color[1] > 0 || color[2] > 0) {
                ledsOnCount++;
            }
            if (ledsOnCount > 30) {
                buffer.setRGB(i, 0, 0, 0);
            } else {
                buffer.setRGB(i, color[0], color[1], color[2]);
            }
        }
        leds.setData(buffer);
    }

    public void setUniformColor(int r, int g, int b){
        iterateAllLeds((index) -> {
            return new Integer[] {r, g, b};
        });
    }
    
    public void updateIdleAnimation() {
        timer_idle ++;
        if (timer_idle >= period) {
            timer_idle = 0;
        } else {
            return;
        }

        counter_idle += direction_idle;
        if (counter_idle == ledCount || counter_idle == -1) {
            direction_idle = -direction_idle;
            counter_idle += 2 * direction_idle;
        }
        iterateAllLeds((index) -> {
            double percent = buffer.getLED(index).red - minPower / 1d / maxPower - decay;
            percent = Math.max(percent, 0);

            if (index == counter_idle) {
                percent = 1.0;
            }

            final double power = (percent * (maxPower - minPower)) + minPower;
            return new Integer[] { (int)power, 0, 0 };
        });
    }

    private void updateAlignAnimation() {
        timer_align ++;
        if (timer_align >= period) {
            timer_align = 0;
        } else {
            return;
        }

        position_align++;
        if (position_align == ledCount / 2) {
            position_align = 0;
        }
        iterateAllLeds((index) -> {
            final boolean powered = 
                index == position_align || // the current position
                index == ledCount-1 - position_align || // the opposite side
                index == position_align - 1 || // the left of the current position
                index == ledCount-1 - position_align + 1; // the right of the opposite side

            if (powered) {
                return new Integer[] { 0xA5, 0x10, 0x90 };
            } else {
                return new Integer[] { 0, 0, 0 };
            }
        }); 
    }
    
    public void update(boolean aligning, boolean inRange) {
        AnimationState state = AnimationState.Idle;

        if (inRange) {
            state = AnimationState.InRange;
        } else if (aligning) {
            state = AnimationState.Aligning;
        } else if (!reflectiveSensor.get()) {
            state = AnimationState.ControllingNote;
        }

        switch (state) {
            case ControllingNote:
                setUniformColor(0, 150, 0);
                break;
            case Aligning:
                updateAlignAnimation();
                break;
            case InRange:
                timer_align = 0;
                position_align = 0;
                setUniformColor(0xA5, 0x10, 0x90);
                break;
            case Idle:
            default:
                updateIdleAnimation();
                break;
        }
    }

}
