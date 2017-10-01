#ifndef __RMSE_H
#define __RMSE_H

class RMSE {
private:
  static const VectorXd zero;
  
  VectorXd accum;
  int count;
  
public:
  RMSE(): count(0), accum(zero) { }

  void Push(const VectorXd& estimate, const VectorXd& truth) {
    VectorXd error = (estimate - truth).array().square();
    
    accum = accum + error;
    count++;
  }

  VectorXd Get() const {
    return (count == 0) ? zero : (accum/count).array().sqrt();
  }
};

const VectorXd RMSE::zero = VectorXd::Zero(4);

#endif
