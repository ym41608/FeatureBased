function Ainv = IPPE_inv33(A)
%computes the inverse of a 3x3 matrix, assuming it is full-rank.
a11 = A(1,1);
a12 = A(1,2);
a13 = A(1,3);

a21 = A(2,1);
a22 = A(2,2);
a23 = A(2,3);

a31 = A(3,1);
a32 = A(3,2);
a33 = A(3,3);

Ainv = [[ a22*a33 - a23*a32, a13*a32 - a12*a33, a12*a23 - a13*a22];
    [ a23*a31 - a21*a33, a11*a33 - a13*a31, a13*a21 - a11*a23];
    [ a21*a32 - a22*a31, a12*a31 - a11*a32, a11*a22 - a12*a21]];
Ainv = Ainv./(a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31);
