%process_results


R1=dlmread('../result1.csv',',');
figure(1);
plot(R1(:,1),R1(:,2:4))
size(R1)
mean(R1(:,2))
mean(R1(:,3))
mean(R1(:,4))


R2=dlmread('../result2.csv',',');
figure(2);
plot(R2(:,1),R2(:,2:4))
size(R2)
mean(R2(:,2))
mean(R2(:,3))
mean(R2(:,4))

R3=dlmread('../result3.csv',',');
figure(3);
plot(R3(:,1),R3(:,2:4))
size(R3)
mean(R3(:,2))
mean(R3(:,3))
mean(R3(:,4))