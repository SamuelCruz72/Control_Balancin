float computeController(float e, float umin, float umax){
    // This function computes the control law for a discretized controller Cd(z)
    // e = ref - y is the current error in the controlled system
    // and umin and umax are the limits of the control signal

    // using the following state space representation
    //       x[n+1]= Ab*x[n] + Bb*e[n]
    //       u[n]= sat(C*x[n] + D*e[n])
    // where  Ab = A - L*C and Bb = B - L*D
    // with  L the gain of a Kalman Filter

    // The following constants define the Ab matrix
    const float aw11 = 0.99551284313201904297;
    const float aw12 = 1.81502079963684082031;
    const float aw21 = -0.00448718527331948280;
    const float aw22 = 0.81502085924148559570;

    // The following constants define the Bb matrix
    const float bw1 = 10.77781677246093750000;
    const float bw2 = -0.53589177131652832031;

    // The following constants define the C matrix
    const float c1 = 0.01233259029686450958;
    const float c2 = -4.98840761184692382812;

    // The following constant define the D scalar
    const float d1 = -14.07446575164794921875;

    // The following constants define the antwindup vector L
    const float l1 = 0.36384773254394531250;
    const float l2 = 0.36384773254394531250;

    // The following variables represent the states x[n]
    // in the state-space representation. They must be declared
    // as static to retain their values between function calls.
    static float x1 = 0;
    static float x2 = 0;

    // The following variables are the new computed states x[n+1]
    // of the state space representation
    float x1_new = 0;
    float x2_new = 0;

    // The following variable is the control signal. 
    // it also must be declared as static to retain its value between function calls. 
    static float u = 0;

    /*************************************************
                THIS IS THE CONTROLLER'S CODE
    **************************************************/

    // Compute the new predicted state x[n+1] = Ab*x[n] + Bb*e[n] + L*u[n]
    x1_new = aw11*x1 + aw12*x2 + bw1*e + l1*u;
    x2_new = aw21*x1 + aw22*x2 + bw2*e + l2*u;

    // Compute the control output u[n] = C*x[n] + D*e[n]
    u = c1*x1 + c2*x2 + d1*e;

    // Saturate control signal
    u = constrain(u, umin, umax);

    // Make the predicted state the current state: x[n] <- x[n+1]
    x1 = x1_new;
    x2 = x2_new;

    // now, the control signal is available to the main control routine
    return u;
}
