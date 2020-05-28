function vect=normaliseAngle(vect)

for i=1:length(vect),
    while vect(i)>pi, 
	vect(i)=vect(i)-2*pi; %si vect(i)=2pi alors vect(i)=0
    end;

    while vect(i)<=-pi,
	vect(i)=vect(i)+2*pi;
    end;
end;

