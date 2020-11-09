#include <iostream>
#include <cmath>
#include <string>

class PID{
  private:
    double Kp;
    double Ti;
    double Td;
    double Ts;
    double maxTh;
    double minTh;
    double y[3];
    double y_bar[3];
    double u[3];
    double c[6]; //math terms that came up while solving difference equation
    double counter = 0;

  public:
    PID(double K_p, double K_i, double K_d, double T_s, double G, double max_Th, double min_Th){
      Kp = K_p;
      Ti = K_p/K_i;
      Td = K_d/K_p;
      Ts = T_s;
      maxTh = max_Th;
      minTh = min_Th;
      y[2] = y_bar[2] = y[1] = y_bar[1] = y[0] = y_bar[0] = 0;
      u[2] = u[1] = u[0] = 0;
      c[0] = 1;
      c[1] = (4*G/(Ts + 2*G));
      c[2] = (Ts - 2*G)/(Ts + 2*G);
      c[3] = (2*G - Ts + Ts*Ts/(2*Ti) - G*Ts/Ti + 2*Td)/(Ts + 2*G);
      c[4] = (Ts*Ts/Ti - 4*G - 4*Td)/(Ts + 2*G);
      c[5] = 1 + Ts/(2*Ti) + 2*Td/(Ts + 2*G);
    }
    //double Update(double y_new, double y_bar_new);
    void print();
    int test = 1;
    double Update(double y_new, double y_bar_new){
      y[2] = y[1];
      y[1] = y[0];
      y[0] = y_new;

      y_bar[2] = y_bar[1];
      y_bar[1] = y_bar[0];
      y_bar[0] = y_bar_new;

      u[2] = u[1];
      u[1] = u[0];

      u[0] = (c[1]*u[1] + c[2]*u[2] + Kp * (c[3]*(y_bar[2] - y[2]) + c[4]*(y_bar[1] - y[1]) + c[5]*(y_bar[0] - y[0])))/c[0];
      
      if (1)
      {
        //std::cout << "C[1]: " << c[1] << "  ";
        //std::cout << "C[2]: " << c[2] << "  ";
        //std::cout << "U[1]: " << u[1] << "\n";
      }
      counter++;
      
      //std::cout << "TERM 1: " << c[1]*u[1] << "\n";
      //std::cout << "U[1]: " << u[1] << "\n";
      //std::cout << "TERM 2: " << c[2]*u[2] << "\n";
      //std::cout << "TERM 3: " << Kp * c[3]*(y_bar[2] - y[2]) << "\n";
      //std::cout << "TERM 4: " << Kp * c[4]*(y_bar[1] - y[1]) << "\n";
      //std::cout << "TERM 5: " << Kp * c[5]*(y_bar[0] - y[0]) << "\n";

      if(u[0] > maxTh){
        u[0] = maxTh;
      }

      else if(u[0] < minTh){
        u[0] = minTh;
      }
      return u[0];
    }
};

