E1 = [ 20 30 -50 ]*pi/180;
C1 = Euler2C(E1,'XYZ');
Q1 = Euler2Q(E1,'XYZ',[1 1 1])

E2 = [ 20 30 50 ]*pi/180;
Q2 = Euler2Q(E2,'XYZ',[1 1 1])
C2 = Euler2C(E2,'XYZ');
