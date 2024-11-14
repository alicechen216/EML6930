% Define adjacency matrix
A_adj = [0 1 1 0 0;
         1 0 1 1 0;
         1 1 0 1 1;
         0 1 1 0 1;
         0 0 1 1 0];

% Create a figure with two subplots
figure;

% Left subplot: Graph representation of A_adj
subplot(1, 2, 1); % 1 row, 2 columns, 1st plot
G = graph(A_adj);
p = plot(G, 'Layout', 'circle', 'NodeColor', 'c', 'EdgeColor', [0.6, 0.6, 0.6], 'LineWidth', 1.5, 'MarkerSize', 7);
p.NodeLabel = {'1', '2', '3', '4', '5'}; % Explicit labels for each node
p.NodeFontSize = 20; % Increase font size of node labels
title('Graph Representation of Agent Connectivity', 'FontSize', 18); % Increased font size for title
highlight(p, 'Edges', 1:numedges(G), 'LineWidth', 2); % Customize edge thickness

% Right subplot: Matrix representation of A_adj in text form
subplot(1, 2, 2); % 1 row, 2 columns, 2nd plot
axis off; % Turn off the axis
title('Adjacency Matrix (A_{adj})', 'FontSize', 24); % Increased font size for title

% Display the matrix elements as text
[nRows, nCols] = size(A_adj);
for row = 1:nRows
    for col = 1:nCols
        text(col, nRows - row + 1, num2str(A_adj(row, col)), 'FontSize', 20, ...
             'HorizontalAlignment', 'center');
    end
end

% Set x and y limits to center the matrix display
xlim([0, nCols + 1]);
ylim([0, nRows + 1]);