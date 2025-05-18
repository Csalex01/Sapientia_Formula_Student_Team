% MATLAB script to create a Simulink model for APPS simulation
% Complies with Formula Student rules T11.8.4-T11.8.12
% Generates a model that can be used for hardware code generation (e.g., STM32)

% Create a new Simulink model
model = 'APPS_Simulation';
new_system(model);
open_system(model);

% Set simulation parameters
set_param(model, 'StopTime', '10'); % 10 seconds simulation
set_param(model, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.001'); % 1 ms step

% Add blocks for pedal input (simulating pedal position 0-100%)
add_block('simulink/Sources/Ramp', [model '/Pedal_Input']);
set_param([model '/Pedal_Input'], 'Slope', '10', 'Start', '0', 'InitialOutput', '0'); % Ramp from 0 to 100% over 10s

% Add sensor models (two sensors with different transfer functions)
% Sensor 1: Linear 0-5V for 0-100% pedal travel
add_block('simulink/Commonly Used Blocks/Gain', [model '/Sensor_1']);
set_param([model '/Sensor_1'], 'Gain', '0.05'); % 0-100% -> 0-5V
add_block('simulink/Commonly Used Blocks/Scope', [model '/Sensor_1_Scope']);
add_line(model, 'Pedal_Input/1', 'Sensor_1/1');
add_line(model, 'Sensor_1/1', 'Sensor_1_Scope/1');

% Sensor 2: Linear 0-2.5V for 0-100% pedal travel
add_block('simulink/Commonly Used Blocks/Gain', [model '/Sensor_2']);
set_param([model '/Sensor_2'], 'Gain', '0.025'); % 0-100% -> 0-2.5V
add_block('simulink/Commonly Used Blocks/Scope', [model '/Sensor_2_Scope']);
add_line(model, 'Pedal_Input/1', 'Sensor_2/1');
add_line(model, 'Sensor_2/1', 'Sensor_2_Scope/1');

% Add plausibility check (>10% deviation detection)
add_block('simulink/Commonly Used Blocks/Math', [model '/Difference']);
set_param([model '/Difference'], 'Operator', 'Abs');
add_block('simulink/Commonly Used Blocks/Math', [model '/Subtract']);
add_line(model, 'Sensor_1/1', 'Subtract/1');
add_line(model, 'Sensor_2/1', 'Subtract/2');
add_line(model, 'Subtract/1', 'Difference/1');

% Convert voltage difference to pedal travel percentage
% Sensor 1: 5V = 100%, Sensor 2: 2.5V = 100% -> Normalize to percentage
add_block('simulink/Commonly Used Blocks/Gain', [model '/To_Percentage']);
set_param([model '/To_Percentage'], 'Gain', '20'); % 1V difference = 20% pedal travel
add_line(model, 'Difference/1', 'To_Percentage/1');

% Check if deviation > 10%
add_block('simulink/Logic and Bit Operations/Compare To Constant', [model '/Plausibility_Check']);
set_param([model '/Plausibility_Check'], 'Operator', '>', 'Const', '10');
add_line(model, 'To_Percentage/1', 'Plausibility_Check/1');

% Add 100 ms persistence check using a delay
add_block('simulink/Discrete/Unit Delay', [model '/Delay_100ms']);
set_param([model '/Delay_100ms'], 'SampleTime', '0.1', 'InitialCondition', '0');
add_block('simulink/Logic and Bit Operations/Logical Operator', [model '/Persist_Check']);
set_param([model '/Persist_Check'], 'Operator', 'AND');
add_line(model, 'Plausibility_Check/1', 'Delay_100ms/1');
add_line(model, 'Plausibility_Check/1', 'Persist_Check/1');
add_line(model, 'Delay_100ms/1', 'Persist_Check/2');

% Shutdown logic: If implausible, set output to 0
add_block('simulink/Signal Routing/Switch', [model '/Shutdown_Switch']);
add_block('simulink/Sources/Constant', [model '/Zero_Output']);
set_param([model '/Zero_Output'], 'Value', '0');
add_line(model, 'Persist_Check/1', 'Shutdown_Switch/2'); % Control input
add_line(model, 'Sensor_1/1', 'Shutdown_Switch/1'); % Normal output (use Sensor 1 as reference)
add_line(model, 'Zero_Output/1', 'Shutdown_Switch/3'); % Shutdown output

% Fully released pedal check (output = 0 if pedal < 1%)
add_block('simulink/Logic and Bit Operations/Compare To Constant', [model '/Released_Check']);
set_param([model '/Released_Check'], 'Operator', '<', 'Const', '1');
add_block('simulink/Signal Routing/Switch', [model '/Released_Switch']);
add_line(model, 'Pedal_Input/1', 'Released_Check/1');
add_line(model, 'Released_Check/1', 'Released_Switch/2');
add_line(model, 'Shutdown_Switch/1', 'Released_Switch/1');
add_line(model, 'Zero_Output/1', 'Released_Switch/3');

% Output scope for torque/throttle
add_block('simulink/Commonly Used Blocks/Scope', [model '/Output_Scope']);
add_line(model, 'Released_Switch/1', 'Output_Scope/1');

% Save and open the model
save_system(model);
open_system(model);

disp('APPS Simulation model created successfully. Run the model to simulate.');