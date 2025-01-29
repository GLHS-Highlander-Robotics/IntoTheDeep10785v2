package subsystem.numbers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private ElapsedTime runtime = new ElapsedTime();
    public double P, I, D, error=0, lastError=0, time, lasttime, deltaError, deltaTime, errorSum=0, errorChange;
    public int targetPos;

    public void start(){
        runtime.reset();
        errorSum=0;
    }

    public void update(int currentEncoderValue){
        lastError = error;
        lasttime = time;
        error = targetPos - currentEncoderValue;
        time = runtime.milliseconds();
        deltaError = error - lastError;
        deltaTime = time - lasttime;
        errorSum+=deltaError*deltaTime;
        errorChange = deltaError/deltaTime;
    }

    public PID(double p, double i, double d){
        P=p;
        I=i;
        D=d;
    }

    public void calculate(){
        return P*error + I*errorSum + D*errorChange;
    }

    public void setTarg(int targetpos){
        targetPos = targetpos;
        errorSum = 0;
        lasttime = 0;
        lastError = 0;
    }
}
