start_values = [
    [0.180000 2.880000 0.540000 3.040000 2.640000],
    [0.560000 1.960000 0.760000 3.220000 4.360000],
    [1.340000 2.630000 1.340000 4.090000 4.840000],
    [1.440000 5.600000 2.530000 6.270000 5.990000],
    [1.440000 0.100000 2.750000 1.870000 0.500000],
    [0.340000 2.120000 2.100000 4.030000 0.590000],
    [1.660000 5.990000 6.140000 2.870000 0.270000],
    [1.600000 0.220000 2.570000 0.820000 1.860000],
    [1.610000 1.360000 5.650000 2.830000 5.260000],
    [0.980000 2.410000 3.780000 1.470000 2.070000],
    [1.170000 1.910000 5.970000 2.460000 3.440000],
    [0.600000 1.870000 4.140000 0.120000 1.860000],
    [1.570000 1.660000 0.190000 1.800000 4.690000],
    [0.470000 1.610000 1.020000 0.660000 3.400000],
    [0.430000 2.210000 3.000000 1.720000 5.650000],
    [0.380000 1.560000 2.120000 4.540000 1.710000],
    [1.220000 0.760000 0.360000 3.340000 4.440000],
    [1.310000 1.290000 1.130000 5.650000 5.500000],
    [0.720000 1.740000 1.720000 5.020000 1.510000],
    [0.690000 1.320000 2.720000 2.550000 3.260000],
    [1.020000 2.460000 1.070000 5.950000 0.710000],
    [1.810000 2.050000 1.140000 0.210000 3.680000],
    [0.740000 1.380000 2.550000 5.740000 2.260000],
    [1.020000 2.030000 5.750000 1.990000 2.930000],
    [0.820000 0.200000 3.170000 5.860000 0.760000],
    [0.810000 2.200000 5.210000 1.970000 2.710000], 
    [0.450000 2.920000 0.730000 2.180000 5.300000], 
    [0.720000 2.150000 3.510000 3.040000 6.010000], 
    [0.860000 1.570000 0.880000 3.240000 5.840000], 
    [1.320000 1.280000 0.460000 3.940000 0.820000], 
    [0.490000 6.280000 1.940000 2.560000 0.690000], 
    [1.700000 1.550000 4.700000 2.600000 0.820000], 
    [1.530000 0.060000 2.250000 1.830000 3.610000], 
    [1.820000 2.150000 0.880000 1.330000 1.660000], 
    [0.550000 5.860000 1.090000 2.240000 3.550000], 
    [0.460000 2.210000 1.900000 5.940000 2.470000], 
    [1.500000 2.760000 1.330000 0.150000 0.390000], 
    [0.590000 2.010000 5.610000 0.080000 4.240000], 
    [0.400000 0.100000 1.640000 2.870000 4.790000], 
%     [1.290000 2.780000 1.400000 0.420000 3.610000], 
%     [1.650000 2.690000 0.490000 0.870000 4.500000], 
%     [1.300000 5.760000 2.260000 4.460000 0.210000], 
%     [1.600000 1.200000 5.170000 1.850000 1.750000], 
%     [0.160000 1.600000 3.650000 1.860000 2.710000], 
    [1.570000 1.520000 1.060000 5.520000 3.980000]];
% 
goal_values = [
    [1.780000 0.510000 0.720000 6.030000 3.440000],
    [1.040000 2.960000 2.280000 5.440000 2.200000],
    [1.370000 1.560000 2.350000 0.130000 2.930000],
    [1.340000 5.200000 2.430000 6.170000 5.790000],
    [1.650000 0.930000 6.260000 4.650000 1.120000],
    [1.630000 1.660000 5.630000 3.920000 5.670000],
    [1.450000 5.930000 3.290000 1.260000 4.700000],
    [0.120000 0.770000 3.550000 2.370000 6.000000],
    [1.020000 2.460000 1.070000 5.950000 0.710000],
    [1.810000 2.050000 1.140000 0.210000 3.680000],
    [0.740000 1.380000 2.550000 5.740000 2.260000],
    [1.020000 2.030000 5.750000 1.990000 2.930000],
    [0.820000 0.200000 3.170000 5.860000 0.760000],
    [1.720000 0.200000 2.690000 5.170000 2.380000],
    [0.260000 0.660000 2.090000 3.560000 4.460000],
    [0.160000 2.270000 1.190000 2.640000 4.290000],
    [1.780000 1.390000 1.620000 6.180000 4.640000],
    [1.520000 1.840000 5.930000 3.140000 1.740000],
    [0.630000 2.020000 3.230000 1.620000 5.050000],
    [0.270000 0.210000 2.690000 5.970000 1.080000],
    [1.480000 1.500000 6.150000 3.250000 0.920000], 
    [0.390000 1.020000 4.600000 2.140000 2.190000], 
    [1.560000 1.650000 0.770000 1.660000 2.960000], 
    [1.260000 2.320000 4.940000 1.330000 0.340000], 
    [1.630000 5.800000 3.520000 1.840000 6.060000], 
    [1.500000 0.630000 4.710000 2.170000 5.710000], 
    [1.140000 1.990000 0.560000 5.500000 4.550000], 
    [1.450000 2.690000 0.960000 0.570000 5.170000], 
    [1.680000 1.970000 5.720000 2.120000 3.840000], 
    [0.110000 2.040000 2.680000 1.150000 4.240000], 
    [1.870000 0.190000 3.140000 2.450000 0.680000], 
    [0.460000 1.900000 3.500000 2.780000 1.040000], 
    [1.380000 3.050000 0.310000 3.110000 2.150000], 
    [0.110000 1.580000 3.070000 2.310000 6.230000], 
    [0.870000 1.320000 1.900000 2.910000 3.460000], 
    [0.960000 2.060000 6.270000 2.580000 5.690000], 
    [1.390000 2.370000 3.840000 0.130000 6.180000], 
    [1.700000 5.690000 0.550000 1.470000 4.780000], 
    [0.930000 3.020000 2.380000 4.770000 3.150000], 
    [0.350000 2.060000 1.160000 6.050000 2.830000]];
dofs = 5;
num_of_samples =39;% size(start_values,1);
% start_values = (2*pi()).*rand(100,dofs);
% goal_values = (2*pi()).*rand(100,dofs);
cost_ = zeros(4,num_of_samples);
time_ = zeros(4,num_of_samples);
vertices_ = zeros(4,num_of_samples);
under_5s = zeros(4,num_of_samples);
success = -1*ones(4,num_of_samples);
for k = 1:4
    for i=1:num_of_samples
        [armplanlength,time,cost,vertices] = runtest("map2.txt",start_values(i,:),goal_values(i,:),k-1);
        cost_(k,i) = cost;
        time_(k,i) = time;
        vertices_(k,i) = vertices;
%         under_5s(k,i) = s5;
        success(k,i) = armplanlength~=0;
        i
    end
end
fprintf("cost\n");
mean(cost_,2);
fprintf("\ntime\n")
mean(time_,2);
fprintf("\nvertices\n")
mean(vertices_,2);
fprintf("\n under 5 seconds\n")
mean(under_5s,2);
