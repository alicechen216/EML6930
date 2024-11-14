% Number of agents/controllers
num_agents = 5;

% Generate agent and controller node labels
agents = arrayfun(@(x) sprintf('A%d', x), 1:num_agents, 'UniformOutput', false);
controllers = arrayfun(@(x) sprintf('C%d', x), 1:num_agents, 'UniformOutput', false);

% Initialize edge lists
agent_edges = cell(num_agents, 2);
controller_edges = cell(num_agents, 2);
agent_controller_edges = cell(num_agents, 2);

% Edges between agents (ring topology)
for i = 1:num_agents
    agent_edges{i,1} = agents{i};
    agent_edges{i,2} = agents{mod(i, num_agents) + 1};
end

% Edges between controllers (ring topology)
for i = 1:num_agents
    controller_edges{i,1} = controllers{i};
    controller_edges{i,2} = controllers{mod(i, num_agents) + 1};
end

% Edges between agents and their controllers
for i = 1:num_agents
    agent_controller_edges{i,1} = agents{i};
    agent_controller_edges{i,2} = controllers{i};
end

% Concatenate all edges
all_edges = [agent_edges; controller_edges; agent_controller_edges];

% Edge types
EdgeType = [repmat({'agent'}, num_agents, 1); ...
            repmat({'control'}, num_agents, 1); ...
            repmat({'interlayer'}, num_agents, 1)];

% Create Edge table
Edges = table(all_edges(:,1), all_edges(:,2), EdgeType, ...
              'VariableNames', {'EndNodes1', 'EndNodes2', 'EdgeType'});

% Create graph
G = graph(Edges.EndNodes1, Edges.EndNodes2);
G.Edges.EdgeType = Edges.EdgeType;

% Assign positions
node_names = G.Nodes.Name;
numnodes = numnodes(G);
pos = zeros(numnodes, 2);
node_colors = zeros(numnodes, 1);

for i = 1:numnodes
    name = node_names{i};
    idx_agent = find(strcmp(agents, name));
    if ~isempty(idx_agent)
        pos(i, :) = [idx_agent, 1];  % Agents at y = 1
        node_colors(i) = 1;          % Color index for agents
    else
        idx_controller = find(strcmp(controllers, name));
        if ~isempty(idx_controller)
            pos(i, :) = [idx_controller, 0];  % Controllers at y = 0
            node_colors(i) = 2;               % Color index for controllers
        end
    end
end

% Plot the graph
figure;
h = plot(G, 'XData', pos(:,1), 'YData', pos(:,2), 'NodeLabel', G.Nodes.Name);
hold on;

% Set node colors
highlight(h, find(node_colors == 1), 'NodeColor', 'skyblue');    % Agents
highlight(h, find(node_colors == 2), 'NodeColor', 'lightgreen'); % Controllers

% Edge indices by type
idx_agent_edges = find(strcmp(G.Edges.EdgeType, 'agent'));
idx_control_edges = find(strcmp(G.Edges.EdgeType, 'control'));
idx_interlayer_edges = find(strcmp(G.Edges.EdgeType, 'interlayer'));

% Set edge colors and styles
highlight(h, 'Edges', idx_agent_edges, 'EdgeColor', 'blue', 'LineStyle', '-');
highlight(h, 'Edges', idx_control_edges, 'EdgeColor', 'green', 'LineStyle', '-');
highlight(h, 'Edges', idx_interlayer_edges, 'EdgeColor', [0.5, 0.5, 0.5], 'LineStyle', '--');

% Add legend
legend({'Agents', 'Controllers'}, 'Location', 'best');

% Remove axis
axis off;

% Add title
title('Networked Multi-Agent System');