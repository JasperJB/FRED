% Define file groups for TensorFlow, PyTorch, and Q-learning runs
tf_files = {'tf1.csv', 'tf2.csv', 'tf3.csv', 'tf4.csv', 'tf5.csv'};
pt_files = {'pt1.csv', 'pt2.csv', 'pt3.csv', 'pt4.csv', 'pt5.csv'};
ql_files = {'ql1.csv', 'ql2.csv', 'ql3.csv', 'ql4.csv', 'ql5.csv'};

% Helper function to load a single metric (column) from multiple CSV files
function metric_values = load_metric(file_list, col_idx)
    n = length(file_list);
    metric_values = zeros(n, 1);
    for i = 1:n
        data = csvread(file_list{i}); 
        % Adjust the row/column access based on how your CSV is structured:
        % - If each file has only one row, you may need data(1, col_idx).
        % - If the file has multiple rows and the metric is an average, adjust accordingly.
        metric_values(i) = data(1, col_idx); 
    end
end

% Load each metric (assuming columns are: 1=CPU, 2=Memory, 3=Power, 4=Time)
tf_cpu = load_metric(tf_files, 1);
pt_cpu = load_metric(pt_files, 1);
ql_cpu = load_metric(ql_files, 1);

tf_mem = load_metric(tf_files, 2);
pt_mem = load_metric(pt_files, 2);
ql_mem = load_metric(ql_files, 2);

tf_pwr = load_metric(tf_files, 3);
pt_pwr = load_metric(pt_files, 3);
ql_pwr = load_metric(ql_files, 3);

tf_time = load_metric(tf_files, 4);
pt_time = load_metric(pt_files, 4);
ql_time = load_metric(ql_files, 4);

% Combine data into grouped arrays
cpu_data  = [tf_cpu, pt_cpu, ql_cpu];
mem_data  = [tf_mem, pt_mem, ql_mem];
pwr_data  = [tf_pwr, pt_pwr, ql_pwr];
time_data = [tf_time, pt_time, ql_time];

% Plot CPU load
figure;
bar(cpu_data);
legend('TensorFlow','PyTorch','Q-Learning','location','northwest');
title('CPU Load Comparison');
xlabel('Trial');
ylabel('CPU Load (%)');

% Plot Memory usage
figure;
bar(mem_data);
legend('TensorFlow','PyTorch','Q-Learning','location','northwest');
title('Memory Usage Comparison');
xlabel('Trial');
ylabel('Memory (units)');

% Plot Power usage
figure;
bar(pwr_data);
legend('TensorFlow','PyTorch','Q-Learning','location','northwest');
title('Power Consumption Comparison');
xlabel('Trial');
ylabel('Power (W/h avg)');

% Plot Real-World Time to Complete
figure;
bar(time_data);
legend('TensorFlow','PyTorch','Q-Learning','location','northwest');
title('Completion Time Comparison');
xlabel('Trial');
ylabel('Time (seconds)');
