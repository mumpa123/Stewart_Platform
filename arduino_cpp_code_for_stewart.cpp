#include <iostream>
#include <cmath>
#include <vector>
// #include <stdio.h>

using namespace std;

double rp = 0.1016; //meters platform radius
double rb = 0.04445; //meters base radius

double sal = 0.02; //meters
double s = 0.1; //meters length of operating leg


//------------------------

double co = 3.14159/180;



// #Anchor points
// #These are for the base and platform in their respective frameworks
// # the coordinates of where the legs attach to the servo and platform respectively
// # six legs so six sets of coords for both base and platform

vector<double> p1{rp*cos(0*co),rp*sin(0*co),0.1};
vector<double> p2{rp*cos(60*co),rp*sin(60*co),0.1};
vector<double> p3{rp*cos(120*co),rp*sin(120*co),0.1};
vector<double> p4{rp*cos(180*co),rp*sin(180*co),0.1};
vector<double> p5{rp*cos(240*co),rp*sin(240*co),0.1};
vector<double> p6{rp*cos(300*co),rp*sin(300*co),0.1};

vector<double> b1 = {rb*cos(0),rb*sin(0),0};
vector<double> b2 = {rb*cos(60*co),rb*sin(60*co),0};
vector<double> b3 = {rb*cos(120*co),rb*sin(120*co),0};
vector<double> b4 = {rb*cos(180*co),rb*sin(180*co),0};
vector<double> b5 = {rb*cos(240*co),rb*sin(240*co),0};
vector<double> b6 = {rb*cos(300*co),rb*sin(300*co),0};



// vector<vector<double>> plat{{rp*cos(0*co),rp*sin(0*co),0.1},{rp*cos(60*co),rp*sin(60*co),0.1},{rp*cos(120*co),rp*sin(120*co),0.1},{rp*cos(180*co),rp*sin(180*co),0.1},{rp*cos(240*co),rp*sin(240*co),0.1},{rp*cos(300*co),rp*sin(300),0.1}};
// vector<vector<double>> base{{rb*cos(0),rb*sin(0),0},{rb*cos(60*co),rb*sin(60*co),0},{rb*cos(120*co),rb*sin(120*co),0},{rb*cos(180*co),rb*sin(180*co),0},{rb*cos(240*co),rb*sin(240*co),0},{rb*cos(300*co),rb*sin(300),0}};
vector<vector<double>> plat = {p1,p2,p3,p4,p5,p6};
vector<vector<double>> base = {b1,b2,b3,b4,b5,b6};

double beta1 = 0*co; //angle of servo arm plane relative to the x-axis
double beta2 = 60*co;
double beta3 = 120*co;
double beta4 = 180*co;
double beta5 = 240*co;
double beta6 = 300*co;

vector<double> betas = {beta1,beta2,beta3,beta4,beta5,beta6};

// vector<double> xp = 
double xp = p1[0];
double yp = p1[1];
double zp = p1[2];

double xb = b1[0];
double yb = b1[1];
double zb = b1[2];

// double xp,yp,zp = p1[0],p1[1],p1[2];
// double xb,yb,zb = b1[0],b2[1],b1[2];

double h0 = sqrt(pow(s,2) + pow(sal,2) - pow((xp - xb),2) - pow((yp - yb),2) - zp);


double L0 = 2*pow(sal,2);
double M0 = 2*sal*(xp - xb);
double N0 = 2*sal*(h0 + zp);
double alpha0 = asin(L0 / (sqrt(pow(M0,2) + pow(N0,2)))) - atan(M0/N0);



//---------------


vector<double> mult_matrix(vector<vector<double>> A ,vector<double> B)
{
    
    vector<double> C;
    double row_sum = 0.0;
    //if () {
    //}
    if (A[0].size() != B.size()){
        cout << "YOU LOSE MULTIPLY FOOL" << endl;
    }
    for (int i = 0; i < A.size(); i++) {
        for (int j = 0; j < 1; j++) {
            for (int k = 0; k < B.size(); k++){
                row_sum += A[i][k] * B[k]; //would require if and then B[k][j] to multiply two matrices
                
            } // end third for
        C.push_back(row_sum);
        row_sum = 0;
        } // end second four
    } // end first for
    return C;
} // end mult_matrix

vector<double> add_matrix(vector<double> A, vector<double> B) {
    
    vector<double> A_B;
    if (A.size() != B.size()) {
        cout << "You lose add!!" << endl;
    } // end if
    else {
        for (int i = 0; i < A.size(); i++) {
            A_B.push_back(A[i] + B[i]);
        } // end for
    } // end else
   
    return A_B;
} // end add_matrix

vector<double> sub_matrix(vector<double> A, vector<double> B){
    
    vector<double> A_B;
    if (A.size() != B.size()){
        cout <<"You Lose subtract!!" << endl;
    }
    else {
        for (int i = 0; i < A.size(); i++){
            A_B.push_back(A[i] - B[i]);
        }

    }
    return A_B;
}

vector<double> leg(vector<double> T, vector<vector<double>> R, vector<double> p, vector<double> b) {
    vector<double> test = mult_matrix(R,p);
    // for (int i = 0; i < test.size(); i++){
    // cout << "test mul = " << test[i] << endl;
    // }
    vector<double> leg_length_vector = sub_matrix(add_matrix(T,mult_matrix(R,p)),b);
    return leg_length_vector;
} // end leg



float pythag(vector<double> vec) {
    float total = 0;
    for (int i = 0; i < vec.size(); i++) {
        cout << "vec value = " << vec[i] << endl;
        total += pow(vec[i], 2); 
    } // end for
    return pow(total, .5);
} // end pythag






int main(){
    
        
    // Rotation Matrix

    float t = 0; // psi
    float a = 0; // theta
    float d = 0; // phi
    float x = 0;
    float y = 0;
    float z = 0;
    float change = 0.001;
    for (int i = 0; i < 1; i++) {
        // cristian stopped and messed up his part anyway
        vector<vector<double>> R = {{cos(t)*cos(a), -sin(t)*cos(d) + cos(t)*sin(a)*sin(d),sin(d)*sin(a) + cos(t)*sin(a)*cos(d)},{sin(t)*cos(a),cos(t)*cos(d) + sin(t)*sin(a)*sin(d),
            -cos(t)*sin(d)+sin(t)*sin(a)*cos(d)},{-sin(a),cos(a)*sin(d),cos(a)*cos(d)}};

        


        // ----------------------------------


        // #Translation Matrix
        // x = 0
        // z = 0
        // y = 0

        vector<double> T = {x,y,z};


        // ----------------------------------


        // #leg lengths

        // #l1 = T + R*p1 - b1
        // #l2 = T + R*p2 ...
        // #...
    

        vector<double> l1 = leg(T,R,p1,b1);
        vector<double> l2 = leg(T,R,p2,b2);
        vector<double> l3 = leg(T,R,p3,b3);
        vector<double> l4 = leg(T,R,p4,b4);
        vector<double> l5 = leg(T,R,p5,b5);
        vector<double> l6 = leg(T,R,p6,b6);

        // #------------------------------------

        
        vector<vector<double>> ls = {l1,l2,l3,l4,l5,l6};
        for (int i = 0; i < ls.size(); i++){
            if (i != 1){

            }
            else{
                cout << "leg " << i << "= " << pythag(ls[i]) << endl;
            }
        }

        // not sure if i was just using this for testing or what
        // for (int i = 0; i < vec.size(ls); i++){
        //     if i != 1:
        //         continue
        //     else:
        //         print('leg', i, '= ', pythag(j))
        // }

        
      
        
        // #Servo Math
        vector<double> alphas;
        for (int i = 0; i < plat.size(); i++){
            // # l = np.sqrt((plat[i][0] - base[i][0])**2 + (plat[i][1] - base[i][1])**2 + (plat[i][2] - base[i][2])**2) #np.sqrt((xp - xb)**2 + (yp - yb)**2 + (zp - zb)**2)
            float l = pythag(ls[i]);
           
            double L = pow(l,2) - (pow(s,2) - pow(sal,2));
         

            double M = 2*sal*(plat[i][2] - base[i][2]);   //2*sal*(zp - zb);

            double N = 2*sal*(cos(betas[i])*(plat[i][0] - base[i][0]) + sin(betas[i])*(plat[i][1] 
            - base[i][1])); //#2*sal*(cos(betas[i])*(xp - xb) + sin(i)*(yp - yb));
            cout << "l = " << l << endl;
            cout << "L = " << L << endl;
            cout << "M = " << M << endl;
            cout << "N = " << N << endl;
            // # print(L)
            // # print(M)
            // # print(N)
            // # print(np.sqrt(M**2 + N**2))s
            double val = ((L / sqrt(pow(M,2) + pow(N,2)))) - (atan((N/M)) );
            cout << "val = " << val << endl;
            if (val > 1 | val < -1){
                double testval = N/M;
                cout << "N/M = " << testval << endl;;
                cout << "oh no. not good val error" << endl;
        }
            alphas.push_back( asin((((L / (sqrt(pow(M,2) + pow(N,2)))) - (atan((N/M)) )) )));
}
        // print('alpha = ', alphas[1]) // not sure what i was printing this out for probably testing
        // cout << "alpha = " << alphas[1] << endl; // prints out alpha value for leg 2 i think
        if (x > 0.2 | x < -0.2){
            change *= -1;
        }
        x += change;
        // sleep(1)
        // ithink delay(10) or whatever for arduino
        // #-----------------------------------
        // # write pos to servo pos being alpha i believe
    



    // # print(l1)
    // # print(l2)
    // cout << l1 << endl; cant print that cause c++ dumb. sucks to suck c++
    // # print(l3)
    // # print(l4)
    // # print(l5)
    // # print(l6)

    // # print(h0)
    

    }

   
    return 0;
}

