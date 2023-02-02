#include <iostream>
#include <cmath>

using namespace std;


double rp = 0.1016;
double rb = 0.04445;

double sal = 0.02;
double s = 0.1;

//-------------------

double co = 3.14159/180;

double p1 [3] = {rp*cos(0*co), rp*sin(0*co), 0.1};
double p2 [3] = {rp*cos(60*co),rp*sin(60*co),0.1};
double p3 [3] = {rp*cos(120*co),rp*sin(120*co),0.1};
double p4 [3] = {rp*cos(180*co),rp*sin(180*co),0.1};
double p5 [3] = {rp*cos(240*co),rp*sin(240*co),0.1};
double p6 [3] = {rp*cos(300*co),rp*sin(300*co),0.1};


double b1 [3] = {rb*cos(0),rb*sin(0),0};
double b2 [3] = {rb*cos(60*co),rb*sin(60*co),0};
double b3 [3] = {rb*cos(120*co),rb*sin(120*co),0};
double b4 [3] = {rb*cos(180*co),rb*sin(180*co),0};
double b5 [3] = {rb*cos(240*co),rb*sin(240*co),0};
double b6 [3] = {rb*cos(300*co),rb*sin(300*co),0};



double plat [6][3] = {{rp*cos(0*co), rp*sin(0*co), 0.1}, {rp*cos(60*co),rp*sin(60*co),0.1}, {rp*cos(120*co),rp*sin(120*co),0.1}, {rp*cos(180*co),rp*sin(180*co),0.1}, {rp*cos(240*co),rp*sin(240*co),0.1}, {rp*cos(300*co),rp*sin(300*co),0.1}};
double base [6][3] = {{rb*cos(0*co), rb*sin(0*co), 0.1}, {rb*cos(60*co),rb*sin(60*co),0.1}, {rb*cos(120*co),rb*sin(120*co),0.1}, {rb*cos(180*co),rb*sin(180*co),0.1}, {rb*cos(240*co),rb*sin(240*co),0.1}, {rb*cos(300*co),rb*sin(300*co),0.1}};


double beta1 = 0*co; //angle of servo arm plane relative to the x-axis
double beta2 = 60*co;
double beta3 = 120*co;
double beta4 = 180*co;
double beta5 = 240*co;
double beta6 = 300*co;

double betas [6] = {beta1,beta2,beta3,beta4,beta5,beta6};

double xp = p1[0];
double yp = p1[1];
double zp = p1[2];

double xb = b1[0];
double yb = b1[1];
double zb = b1[2];

double h0 = sqrt(pow(s,2) + pow(sal,2) - pow((xp - xb), 2) - pow((yp - yb),2) - zp);

double L0 = 2*pow(sal,2);
double M0 = 2*sal*(xp-xb);
double N0 = 2*sal*(h0+zp);
double alpha0 = asin(L0 / (sqrt(pow(M0,2) + pow(N0,2)))) - atan(M0/N0);


//----------

double* mult_matrix(double A[3][3], double B[3]){
    // double C[3];
    static double C[3]; //array declared as static
    double row_sum = 0;
    for (int i=0; i<3;i++){
        for (int j=0; j<3; j++){
            row_sum += A[i][j] * B[j];
        C[i] = row_sum;
        }
    }
    return C;
}

double* add_matrix(double A[3], double B[3]){
    static double A_B[3];
    for (int i=0; i<3; i++){
        A_B[i] = A[i] + B[i];
    }
    return A_B;
}

double* sub_matrix(double A[3], double B[3]){
    static double A_B[3];
    for (int i=0; i<3; i++){
        A_B[i] = A[i] - B[i];
    }
    return A_B;
}


double* leg(double T[3], double R[3][3], double p[3], double b[3]){
    double t1_vect[3];
    double t2_vect[3];
    static double t3_vect[3];
    double* t1;
    double* t2;
    double* t3;
    t1 = t1_vect;
    t2 = t2_vect;
    t3 = t3_vect;
    static double leg_length_vector[3];
    t1 = mult_matrix(R,p);
    cout << "t1 = " << *t1 << endl;
    t2 = add_matrix(T,t1);
    cout << "t2 = " << *t2 << endl;
    t3 = sub_matrix(t2,b);
    cout << "T3 = " << *t3 << endl;
    

    return t3_vect;
}

double pythag(double vec[3]){
    double total = 0;
    for (int i=0; i<3; i++){
        total += pow(vec[i],2);
    }
    return pow(total, 0.5);
}






// double test [3] = {1,2,3};


double t = 0; // psi
double a = 0; // theta
double d = 0; // phi
double x = 0;
double y = 0;
double z = 0;
double change = 0.001;

int main(){
    /*  TEST CODE
    cout << "test" << endl;
    for (int i = 0; i < 6; i++){
        cout << plat[i][0] << endl;
    }
    double* ans_test; //pointer to hold address
    ans_test = mult_matrix(R,test);
    for (int i=0; i<3; i++){
        cout << ans_test[i] << endl;
    }

    cout << "|||" << endl;
    double* test2_ans;
    test2_ans = leg(test,R,test,test);
    for (int i=0; i<3; i++){
        cout << test2_ans[i] << endl;
    }
    */

       
    // Rotation Matrix

    for (int test=0; test<1; test++){
    double R [3][3] = {{cos(t)*cos(a), -sin(t)*cos(d) + cos(t)*sin(a)*sin(d),sin(d)*sin(a) + cos(t)*sin(a)*cos(d)},
                       {sin(t)*cos(a),cos(t)*cos(d) + sin(t)*sin(a)*sin(d),-cos(t)*sin(d)+sin(t)*sin(a)*cos(d)},
                       {-sin(a),cos(a)*sin(d),cos(a)*cos(d)}};
    

    
    double T [3] = {x,y,z};

    double* l1;
    l1 = leg(T,R,p1,b1);
    double* l2;
    l2 = leg(T,R,p2,b2);
    double* l3;
    l3 = leg(T,R,p3,b3);
    double* l4;
    l4 = leg(T,R,p4,b4);
    double* l5;
    l5 = leg(T,R,p5,b5);
    double* l6;
    l6 = leg(T,R,p6,b6);

    cout << "l1 = " << *l1 << endl;
    cout << "l2 = " << *l2 << endl;

    double* ls [6] = {l1,l2,l3,l4,l5,l6}; //problem could arise here not sure 

    for (int c=0; c<6; c++){
        cout << "l = " <<  ls[c] << endl;
    }


    double alphas[6];
    for (int i=0; i<6; i++){
        double l = pythag(ls[i]);
        cout << "ls value " << ls[i] << endl;
        cout << "l1 value " << l1 << endl;
        double L = pow(l,2) - (pow(s,2) - pow(sal,2));
        double M = 2*sal*(plat[i][2] - base[i][2]);
        double N = 2*sal*(cos(betas[i])*(plat[i][0] - base[i][0]) + sin(betas[i])*(plat[i][1] - base[i][1]));
        cout << "l = " << l << endl;
        cout << "L = " << L << endl;
        cout << "M = " << M << endl;
        cout << "N = " << N << endl;
        double val = ((L / sqrt(pow(M,2) + pow(N,2)))) - (atan((N/M)));
        
        // if (val > 1 | val < -1){
        //     // cout << val << endl;
        //     // cout << "value error" << endl;

        // }

        alphas[i] = asin((((L/(sqrt(pow(M,2) + pow(N,2)))) - (atan((N/M))))));
    } //end alpha for loop
    
    if (x > 0.2 | x < -0.2){
        change *= -1;
    }

    x += change;

    cout << "Alpha = " << alphas[1] << endl;
    }
    return 0;
}


