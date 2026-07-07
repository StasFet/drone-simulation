public class PIDFController {
    public float Kp;
    public float Ki;
    public float Kd;
    public float Kf;
    private float integral;
    private float lastError;
    private float prevMeasurement;

    // for debugging
    public float pVal = 0f;
    public float iVal = 0f;
    public float dVal = 0f;

    public PIDFController(float kp = 0, float ki = 0, float kd = 0, float kf = 0) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        Kf = kf;
        integral = 0f;
        lastError = 0f;
        prevMeasurement = 0f;
    }
    public float Update(float target, float current, float deltaTime) {
        float error = target - current;
        integral += error * deltaTime;
        float derivative = - (current - prevMeasurement) / deltaTime;
        lastError = error;
        pVal = Kp * error;
        iVal = Ki * integral;
        dVal = Kd * derivative;
        prevMeasurement = current;
        return pVal + iVal + dVal + Kf;
    }

    public float Update(float error, float deltaTime) {
        integral += error * deltaTime;
        float derivative = (error - lastError) / deltaTime;
        lastError = error;
        pVal = Kp * error;
        iVal = Ki * integral;
        dVal = Kd * derivative;
        lastError = error;
        return pVal + iVal + dVal + Kf;
    }
}
