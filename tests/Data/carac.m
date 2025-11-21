

M = readtable("20251119_052022_esp_G2.csv")

%% Params 
t =  table2array(M(:, 1)); 
u =  table2array(M(:, 5)); 
y =  table2array(M(:, 4)); 


%% EDA 
plot(t, u)
plot(t, y)



%% save 
G1ss =  tf(ss1)
G1tf = tf(tf3)


save("ft", "G1tf", "G1ss"); 


%% 
