function Ainv = IPPE_inv22(A)
%computes the inverse of a 2x2 matrix, assuming it is full-rank.
dt = (A(1,1)*A(2,2) - A(1,2)*A(2,1));
Ainv = [  A(2,2)/dt, -A(1,2)/dt;-A(2,1)/dt,  A(1,1)/dt];