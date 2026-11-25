function alignAnyBlocks()
% ALIGNANYBLOCKS 按拓扑秩整理选中子系统所在的连通网络。
% 端口间距保持 Simulink 原生布局；只移动模块位置，不修改子系统内部端口。

    cfg = struct('GapX', 70, 'GapY', 10, 'Pad', 8, ...
        'Passes', 6, 'Tolerance', 1, 'MinHeightScale', 0.5, ...
        'MaxHeightScale', 3.0);

    selected = find_system(gcs, 'SearchDepth', inf, ...
        'Type', 'Block', 'Selected', 'on');
    subsystems = selected(cellfun(@isSubsystem, selected));
    if isempty(subsystems)
        warning('请先选中一个子系统作为布局锚点。');
        return;
    end

    seed = subsystems{1};
    info = collectGraph(get_param(seed, 'Parent'));
    seedIdx = find(strcmp(info.blocks, seed), 1);
    [component, edges] = connectedComponent(info, seedIdx);
    if numel(component) < 2
        warning('选中子系统没有可布局的同层连接。');
        return;
    end

    centerIdx = chooseCenter(info, component, edges, seedIdx);
    rank = topologyRank(info, component, edges);
    [spine, spineEdges] = longestSpine(info, component, edges, rank);
    [straightEdges, branchEdges] = classifyEdges(info, spine);
    inComponent = ismember([straightEdges.src], component) & ...
        ismember([straightEdges.dst], component);
    straightEdges = straightEdges(inComponent);
    branchEdges = branchEdges(ismember([branchEdges.src], component) & ...
        ismember([branchEdges.dst], component));

    fprintf('连接查询：%d 个模块，%d 条连接；可直连 %d，分支/汇合 %d。\n', ...
        numel(component), numel(edges), numel(straightEdges), numel(branchEdges));
    fprintf('中央子系统：%s。\n', get_param(info.blocks{centerIdx}, 'Name'));
    if ~isempty(spine)
        spineNames = cellfun(@(idx) get_param(info.blocks{idx}, 'Name'), ...
            num2cell(spine), 'UniformOutput', false);
        fprintf('最长主轴：%s。\n', strjoin(spineNames, ' -> '));
    end
    targetPitch = referencePitch(info, edges, centerIdx);
    resizePrimarySides(info, component, edges, rank, centerIdx, targetPitch, cfg);
    axisY = portY(spineEdges(1).srcPort);
    fprintf('主连接侧参考端口间距：%.1f px。\n', targetPitch);

    for passIdx = 1:cfg.Passes
        placeColumns(info, component, edges, rank, centerIdx, cfg);
        placeSpine(info, spine, spineEdges, centerIdx, axisY, cfg);
        alignToSpineGrid(info, component, edges, rank, spine);
        alignStraightEdges(info, component, straightEdges, centerIdx, ...
            spine, cfg);
        resolveOverlaps(info, component, rank, centerIdx, cfg);
        alignStraightEdges(info, component, straightEdges, centerIdx, ...
            spine, cfg);
        overlapCount = countOverlaps(info, component, 0);
        portError = straightPortMetrics(straightEdges, cfg.Tolerance);
        if overlapCount == 0 && portError <= cfg.Tolerance
            break;
        end
    end

    correctExactAlignment(info, component, straightEdges, centerIdx, spine);
    arrangeBoundaryPorts(info, component, edges, spine, cfg);
    routeLines(info.lines);
    normalizeBranchTrunks(info, cfg);
    [~, ~, badLineIndices] = lineGeometry(info);
    repairedLines = repairBlockCrossings(info, info.lines(badLineIndices), cfg);
    [diagonalCount, blockHits] = lineGeometry(info);
    [portError, aligned, straightCount] = straightPortMetrics(straightEdges, ...
        cfg.Tolerance);
    overlapCount = countOverlaps(info, component, 0);
    fprintf(['布局完成：重叠 %d，可直连对齐 %d/%d，最大误差 %.1f px，' ...
        '分支 %d，斜线 %d，穿模块 %d，修复线段 %d。\n'], overlapCount, ...
        aligned, straightCount, portError, numel(branchEdges), ...
        diagonalCount, blockHits, repairedLines);
    if overlapCount > 0 || portError > cfg.Tolerance || blockHits > 0
        warning('仍有布局约束未满足，请检查固定模块冲突或未解析的跨层连接。');
    end
end

function tf = isSubsystem(path)
    try
        type = get_param(path, 'BlockType');
        tf = strcmpi(type, 'SubSystem') || strcmpi(type, 'ModelReference');
    catch
        tf = false;
    end
end

function info = collectGraph(scope)
    blocks = find_system(scope, 'SearchDepth', 1, 'Type', 'Block');
    blocks = blocks(~strcmp(blocks, scope));
    blockMap = containers.Map(blocks, num2cell(1:numel(blocks)));
    ports = struct('block', {}, 'handle', {}, 'io', {});
    for b = 1:numel(blocks)
        ph = get_param(blocks{b}, 'PortHandles');
        names = fieldnames(ph);
        for n = 1:numel(names)
            handles = ph.(names{n});
            for k = 1:numel(handles)
                if handles(k) > 0
                    port = struct('block', blocks{b}, 'handle', handles(k), ...
                        'io', portDirection(names{n}));
                    ports(end+1) = port; %#ok<AGROW>
                end
            end
        end
    end

    lines = find_system(scope, 'SearchDepth', 1, ...
        'FindAll', 'on', 'Type', 'Line');
    edges = struct('src', {}, 'dst', {}, 'srcPort', {}, 'dstPort', {}, ...
        'line', {}, 'kind', {});
    for l = 1:numel(lines)
        srcPort = get_param(lines(l), 'SrcPortHandle');
        dstPorts = get_param(lines(l), 'DstPortHandle');
        for s = 1:numel(srcPort)
            if srcPort(s) <= 0
                continue;
            end
            for d = 1:numel(dstPorts)
                if dstPorts(d) <= 0
                    continue;
                end
                srcBlock = get_param(srcPort(s), 'Parent');
                dstBlock = get_param(dstPorts(d), 'Parent');
                if isKey(blockMap, srcBlock) && isKey(blockMap, dstBlock)
                    edge = struct('src', blockMap(srcBlock), ...
                        'dst', blockMap(dstBlock), 'srcPort', srcPort(s), ...
                        'dstPort', dstPorts(d), 'line', lines(l), 'kind', 'signal');
                    edges = addEdge(edges, edge);
                end
            end
        end
    end

    edges = addTaggedEdges(edges, ports, blocks, 'Goto', 'From', ...
        'GotoTag', 'in', 'out', 'goto');
    edges = addTaggedEdges(edges, ports, blocks, 'DataStoreWrite', ...
        'DataStoreRead', 'DataStoreName', 'in', 'out', 'datastore');
    info.blocks = blocks;
    info.blockMap = blockMap;
    info.ports = ports;
    info.edges = edges;
    info.lines = lines;
end

function io = portDirection(fieldName)
    name = lower(fieldName);
    if contains(name, 'out')
        io = 'out';
    elseif contains(name, 'in') || any(strcmpi(fieldName, ...
            {'Enable', 'Trigger', 'Ifaction', 'Reset', 'State', 'Action'}))
        io = 'in';
    else
        io = 'both';
    end
end

function edges = addEdge(edges, edge)
    for k = 1:numel(edges)
        if edges(k).srcPort == edge.srcPort && ...
                edges(k).dstPort == edge.dstPort && ...
                strcmp(edges(k).kind, edge.kind)
            return;
        end
    end
    edges(end+1) = edge;
end

function edges = addTaggedEdges(edges, ports, blocks, srcType, dstType, ...
        property, srcIo, dstIo, kind)
    srcs = blocks(cellfun(@(b) strcmpi(get_param(b, 'BlockType'), srcType), ...
        blocks));
    dsts = blocks(cellfun(@(b) strcmpi(get_param(b, 'BlockType'), dstType), ...
        blocks));
    for s = 1:numel(srcs)
        name = get_param(srcs{s}, property);
        srcPort = firstPort(ports, srcs{s}, srcIo);
        for d = 1:numel(dsts)
            if isempty(name) || ~strcmp(name, get_param(dsts{d}, property))
                continue;
            end
            dstPort = firstPort(ports, dsts{d}, dstIo);
            if srcPort > 0 && dstPort > 0
                edge = struct('src', findBlock(blocks, srcs{s}), ...
                    'dst', findBlock(blocks, dsts{d}), ...
                    'srcPort', srcPort, 'dstPort', dstPort, ...
                    'line', 0, 'kind', kind);
                edges = addEdge(edges, edge);
            end
        end
    end
end

function port = firstPort(ports, blockPath, io)
    port = -1;
    for k = 1:numel(ports)
        if strcmp(ports(k).block, blockPath) && ...
                (strcmp(ports(k).io, io) || strcmp(ports(k).io, 'both'))
            port = ports(k).handle;
            return;
        end
    end
end

function idx = findBlock(blocks, path)
    idx = find(strcmp(blocks, path), 1);
end

function [component, edges] = connectedComponent(info, seedIdx)
    n = numel(info.blocks);
    adjacency = cell(n, 1);
    for k = 1:numel(info.edges)
        e = info.edges(k);
        adjacency{e.src}(end+1) = e.dst;
        adjacency{e.dst}(end+1) = e.src;
    end
    seen = false(n, 1);
    queue = seedIdx;
    seen(seedIdx) = true;
    head = 1;
    while head <= numel(queue)
        current = queue(head);
        head = head + 1;
        next = unique(adjacency{current});
        next = next(next > 0 & next <= n);
        next = next(~seen(next));
        seen(next) = true;
        queue = [queue, next]; %#ok<AGROW>
    end
    component = find(seen);
    keep = seen([info.edges.src]) & seen([info.edges.dst]);
    edges = info.edges(keep);
end

function centerIdx = chooseCenter(info, component, edges, seedIdx)
    bestScore = -1;
    centerIdx = seedIdx;
    for idx = component(:).'
        if ~isSubsystem(info.blocks{idx})
            continue;
        end
        connected = [];
        for e = edges
            if e.src == idx
                connected(end+1) = e.srcPort; %#ok<AGROW>
            elseif e.dst == idx
                connected(end+1) = e.dstPort; %#ok<AGROW>
            end
        end
        total = sum(strcmp({info.ports.block}, info.blocks{idx}));
        score = 1000*numel(unique(connected)) + total;
        if score > bestScore || (score == bestScore && idx == seedIdx)
            bestScore = score;
            centerIdx = idx;
        end
    end
end

function rank = topologyRank(info, component, edges)
    n = numel(info.blocks);
    remaining = false(n, 1);
    remaining(component) = true;
    indegree = zeros(n, 1);
    for e = edges(:).'
        indegree(e.dst) = indegree(e.dst) + (e.src ~= e.dst);
    end
    rank = zeros(n, 1);
    while any(remaining(component))
        roots = find(remaining & indegree == 0);
        if isempty(roots)
            roots = find(remaining, 1);
            indegree(roots) = 0;
        end
        for current = roots(:).'
            remaining(current) = false;
            outgoing = find([edges.src] == current);
            for e = outgoing
                next = edges(e).dst;
                if remaining(next)
                    rank(next) = max(rank(next), rank(current) + 1);
                    indegree(next) = max(0, indegree(next) - 1);
                end
            end
        end
    end
end

function [spine, spineEdges] = longestSpine(info, component, edges, rank)
    n = numel(info.blocks);
    remaining = false(n, 1);
    remaining(component) = true;
    indegree = zeros(n, 1);
    for e = edges(:).'
        if remaining(e.src) && remaining(e.dst) && ...
                rank(e.dst) > rank(e.src)
            indegree(e.dst) = indegree(e.dst) + 1;
        end
    end
    roots = component(indegree(component) == 0);
    if isempty(roots)
        roots = component(:);
    end

    dist = -inf(n, 1);
    previous = zeros(n, 1);
    previousEdge = zeros(n, 1);
    dist(roots) = 0;
    order = sortrows([rank(component), component(:)]);
    for k = 1:size(order, 1)
        current = order(k, 2);
        if ~isfinite(dist(current))
            continue;
        end
        for eIdx = find([edges.src] == current)
            next = edges(eIdx).dst;
            if remaining(next) && rank(next) > rank(current) && ...
                    dist(current) + 1 > dist(next)
                dist(next) = dist(current) + 1;
                previous(next) = current;
                previousEdge(next) = eIdx;
            end
        end
    end

    candidates = component(isfinite(dist(component)));
    if isempty(candidates)
        spine = [];
        spineEdges = struct('src', {}, 'dst', {}, 'srcPort', {}, ...
            'dstPort', {}, 'line', {}, 'kind', {});
        return;
    end
    [~, best] = max(dist(candidates));
    tail = candidates(best);
    nodes = tail;
    pathEdges = [];
    while previous(tail) > 0
        pathEdges = [previousEdge(tail), pathEdges]; %#ok<AGROW>
        tail = previous(tail);
        nodes = [tail, nodes]; %#ok<AGROW>
    end

    spine = nodes;
    spineEdges = edges(pathEdges);
end

function [straightEdges, branchEdges] = classifyEdges(info, spine)
    edges = info.edges(:);
    visual = [edges.line].' > 0;
    srcKey = zeros(numel(edges), 1);
    dstKey = zeros(numel(edges), 1);
    for k = 1:numel(edges)
        srcKey(k) = edges(k).src*1000 + ...
            get_param(edges(k).srcPort, 'PortNumber');
        dstKey(k) = edges(k).dst*1000 + ...
            get_param(edges(k).dstPort, 'PortNumber');
    end
    [~, ~, srcGroup] = unique(srcKey);
    [~, ~, dstGroup] = unique(dstKey);
    srcCount = accumarray(srcGroup, 1);
    dstCount = accumarray(dstGroup, 1);
    special = false(numel(edges), 1);
    for k = 1:numel(edges)
        type = get_param(edges(k).dstPort, 'PortType');
        special(k) = any(strcmpi(type, ...
            {'trigger', 'enable', 'action', 'reset', 'state'}));
    end
    straightMask = visual & (srcCount(srcGroup) == 1) & ...
        (dstCount(dstGroup) == 1) & ~special;
    for k = 1:numel(edges)
        ph = get_param(info.blocks{edges(k).src}, 'PortHandles');
        sourceOnly = ~any(ph.Inport > 0) && any(ph.Outport > 0);
        if sourceOnly && ~ismember(edges(k).src, spine)
            straightMask(k) = false;
        end
    end
    branchMask = visual & ~straightMask;
    straightEdges = edges(straightMask);
    branchEdges = edges(branchMask);
end

function pitch = referencePitch(info, edges, centerIdx)
    field = primaryField(info, edges, centerIdx);
    pitch = NaN;
    try
        innerType = field;
        innerPorts = find_system(info.blocks{centerIdx}, 'SearchDepth', 1, ...
            'BlockType', innerType);
        if numel(innerPorts) > 1
            ys = zeros(1, numel(innerPorts));
            for k = 1:numel(innerPorts)
                pos = get_param(innerPorts{k}, 'Position');
                ys(k) = (pos(2) + pos(4))/2;
            end
            pitch = medianSpacing(ys);
        end
    catch
    end
    if isnan(pitch)
        ph = get_param(info.blocks{centerIdx}, 'PortHandles');
        pitch = medianSpacing(portYs(ph.(field)));
    end
    if isnan(pitch)
        pitch = 45;
    end
end

function resizePrimarySides(info, component, edges, rank, centerIdx, targetPitch, cfg)
    for idx = component(:).'
        field = primaryField(info, edges, idx, rank, centerIdx);
        ph = get_param(info.blocks{idx}, 'PortHandles');
        handles = ph.(field);
        pitch = medianSpacing(portYs(handles));
        if numel(handles) < 2 || isnan(pitch) || abs(pitch-targetPitch) < 0.5
            continue;
        end
        pos = get_param(info.blocks{idx}, 'Position');
        originalHeight = pos(4) - pos(2);
        centerY = (pos(2) + pos(4))/2;
        for iter = 1:4
            newHeight = (pos(4)-pos(2))*targetPitch/pitch;
            newHeight = min(cfg.MaxHeightScale*originalHeight, ...
                max(cfg.MinHeightScale*originalHeight, newHeight));
            set_param(info.blocks{idx}, 'Position', ...
                round([pos(1), centerY-newHeight/2, pos(3), centerY+newHeight/2]));
            ph = get_param(info.blocks{idx}, 'PortHandles');
            pitch = medianSpacing(portYs(ph.(field)));
            if isnan(pitch) || abs(pitch-targetPitch) < 0.5
                break;
            end
            pos = get_param(info.blocks{idx}, 'Position');
        end
    end
end

function field = primaryField(info, edges, idx, rank, centerIdx)
    inPorts = [];
    outPorts = [];
    ph = get_param(info.blocks{idx}, 'PortHandles');
    inHandles = ph.Inport;
    outHandles = ph.Outport;
    for e = edges(:).'
        if e.dst == idx && ismember(e.dstPort, inHandles)
            inPorts(end+1) = e.dstPort; %#ok<AGROW>
        end
        if e.src == idx && ismember(e.srcPort, outHandles)
            outPorts(end+1) = e.srcPort; %#ok<AGROW>
        end
    end
    if nargin >= 5
        if rank(idx) < rank(centerIdx)
            field = 'Outport';
            return;
        elseif rank(idx) > rank(centerIdx)
            field = 'Inport';
            return;
        end
    end
    if numel(unique(inPorts)) >= numel(unique(outPorts))
        field = 'Inport';
    else
        field = 'Outport';
    end
end

function ys = portYs(handles)
    ys = zeros(1, numel(handles));
    for k = 1:numel(handles)
        pos = get_param(handles(k), 'Position');
        ys(k) = pos(2);
    end
end

function pitch = medianSpacing(values)
    values = sort(values(:));
    if numel(values) < 2
        pitch = NaN;
    else
        spacing = diff(values);
        spacing = spacing(spacing > 0);
        if isempty(spacing)
            pitch = NaN;
        else
            pitch = median(spacing);
        end
    end
end

function placeColumns(info, component, edges, rank, centerIdx, cfg)
    positions = getPositions(info, 1:numel(info.blocks));
    centerPos = positions(centerIdx, :);
    centerRank = rank(centerIdx);
    ranks = unique(rank(component));
    lanesByRank = cell(numel(ranks), 1);
    for r = 1:numel(ranks)
        inds = component(rank(component) == ranks(r));
        desired = desiredCenters(info, inds, edges, rank);
        heights = positions(inds, 4) - positions(inds, 2);
        lanesByRank{r} = assignLanes(desired, heights, inds, cfg.GapY);
    end

    leftRanks = sort(ranks(ranks < centerRank), 'descend');
    boundary = centerPos(1) - cfg.GapX;
    for r = leftRanks(:).'
        lanes = lanesByRank{find(ranks == r, 1)};
        for lane = 1:numel(lanes)
            inds = lanes{lane};
            widths = positions(inds, 3) - positions(inds, 1);
            positions(inds, 3) = boundary;
            positions(inds, 1) = boundary - widths;
            boundary = min(positions(inds, 1)) - cfg.GapX;
        end
    end

    rightRanks = sort(ranks(ranks > centerRank));
    boundary = centerPos(3) + cfg.GapX;
    for r = rightRanks(:).'
        lanes = lanesByRank{find(ranks == r, 1)};
        for lane = 1:numel(lanes)
            inds = lanes{lane};
            widths = positions(inds, 3) - positions(inds, 1);
            positions(inds, 1) = boundary;
            positions(inds, 3) = boundary + widths;
            boundary = max(positions(inds, 3)) + cfg.GapX;
        end
    end
    applyPositions(info, component, positions(component, :));

    for r = 1:numel(ranks)
        inds = component(rank(component) == ranks(r));
        desired = desiredCenters(info, inds, edges, rank);
        heights = positions(inds, 4) - positions(inds, 2);
        if ranks(r) == centerRank && ismember(centerIdx, inds)
            desired = fitAroundCenter(desired, heights, inds, centerIdx, ...
                positions, cfg.GapY);
        end
        positions(inds, 2) = round(desired - heights/2);
        positions(inds, 4) = round(desired + heights/2);
        applyPositions(info, inds, positions(inds, :));
    end
end

function desired = desiredCenters(info, inds, edges, rank)
    positions = getPositions(info, inds);
    desired = (positions(:, 2) + positions(:, 4)) / 2;
    for k = 1:numel(inds)
        values = [];
        centerY = desired(k);
        for e = edges(:).'
            if e.src == inds(k) && abs(rank(e.dst) - rank(inds(k))) == 1
                values(end+1) = portY(e.dstPort) - ...
                    (portY(e.srcPort) - centerY); %#ok<AGROW>
            elseif e.dst == inds(k) && abs(rank(e.src) - rank(inds(k))) == 1
                values(end+1) = portY(e.srcPort) - ...
                    (portY(e.dstPort) - centerY); %#ok<AGROW>
            end
        end
        if ~isempty(values)
            desired(k) = median(values);
        end
    end
end

function placeSpine(info, spine, spineEdges, centerIdx, axisY, cfg)
    if numel(spine) < 2 || numel(spineEdges) ~= numel(spine)-1
        return;
    end
    positions = getPositions(info, 1:numel(info.blocks));
    widths = positions(spine, 3) - positions(spine, 1);
    pitchX = max(widths) + cfg.GapX;
    centerSlot = find(spine == centerIdx, 1);
    if isempty(centerSlot)
        centerSlot = ceil(numel(spine)/2);
    end
    centerX = positions(centerIdx, 1) + ...
        (positions(centerIdx, 3)-positions(centerIdx, 1))/2;
    oldCenters = (positions(spine, 2)+positions(spine, 4))/2;
    special = false(1, numel(spineEdges));
    for k = 1:numel(spineEdges)
        type = get_param(spineEdges(k).dstPort, 'PortType');
        special(k) = any(strcmpi(type, ...
            {'trigger', 'enable', 'action', 'reset', 'state'}));
    end
    compensation = find(special, 1, 'last');
    if isempty(compensation)
        compensation = ceil(numel(spineEdges)/2);
    end

    edge1 = spineEdges(1);
    commonY = axisY;
    centersY = nan(1, numel(spine));
    centersY(1) = commonY - (portY(edge1.srcPort)-oldCenters(1));
    for k = 1:compensation-1
        edge = spineEdges(k);
        centersY(k+1) = centersY(k) + ...
            (portY(edge.srcPort)-oldCenters(k)) - ...
            (portY(edge.dstPort)-oldCenters(k+1));
    end

    lastEdge = spineEdges(end);
    centersY(end) = commonY - ...
        (portY(lastEdge.dstPort)-oldCenters(end));
    for k = numel(spineEdges):-1:compensation+1
        edge = spineEdges(k);
        centersY(k) = centersY(k+1) - ...
            (portY(edge.srcPort)-oldCenters(k)) + ...
            (portY(edge.dstPort)-oldCenters(k+1));
    end
    for k = 1:numel(spine)
        idx = spine(k);
        xCenter = centerX + (k-centerSlot)*pitchX;
        width = positions(idx, 3) - positions(idx, 1);
        height = positions(idx, 4) - positions(idx, 2);
        positions(idx, :) = round([xCenter-width/2, centersY(k)-height/2, ...
            xCenter+width/2, centersY(k)+height/2]);
    end
    applyPositions(info, spine, positions(spine, :));
end

function lanes = assignLanes(desired, heights, blockIndices, gap)
    lanes = {};
    laneIntervals = {};
    [~, order] = sort(desired);
    for idx = order(:).'
        interval = [desired(idx)-heights(idx)/2-gap, ...
            desired(idx)+heights(idx)/2+gap];
        assigned = false;
        for lane = 1:numel(lanes)
            if ~any(interval(1) < laneIntervals{lane}(:, 2) & ...
                    laneIntervals{lane}(:, 1) < interval(2))
                lanes{lane}(end+1) = blockIndices(idx); %#ok<AGROW>
                laneIntervals{lane}(end+1, :) = interval; %#ok<AGROW>
                assigned = true;
                break;
            end
        end
        if ~assigned
            lanes{end+1} = blockIndices(idx); %#ok<AGROW>
            laneIntervals{end+1} = interval; %#ok<AGROW>
        end
    end
end

function desired = fitAroundCenter(desired, heights, inds, centerIdx, ...
        positions, gap)
    fixed = find(inds == centerIdx, 1);
    fixedY = (positions(centerIdx, 2)+positions(centerIdx, 4))/2;
    desired(fixed) = fixedY;
    for k = 1:numel(inds)
        if k == fixed
            continue;
        end
        required = (heights(fixed)+heights(k))/2 + gap;
        if abs(desired(k)-fixedY) < required
            if desired(k) <= fixedY
                desired(k) = fixedY - required;
            else
                desired(k) = fixedY + required;
            end
        end
    end
end

function alignStraightEdges(info, component, edges, centerIdx, fixedNodes, cfg)
    edges = edges([edges.line] > 0);
    if isempty(edges)
        return;
    end
    positions = getPositions(info, 1:numel(info.blocks));
    centers = (positions(:, 2) + positions(:, 4))/2;
    srcOffset = zeros(1, numel(edges));
    dstOffset = zeros(1, numel(edges));
    for k = 1:numel(edges)
        srcOffset(k) = portY(edges(k).srcPort) - centers(edges(k).src);
        dstOffset(k) = portY(edges(k).dstPort) - centers(edges(k).dst);
    end

    for passIdx = 1:cfg.Passes
        currentScore = score(centers, edges, srcOffset, dstOffset, cfg.Tolerance);
        bestGain = 0;
        bestIdx = 0;
        bestCenter = 0;
        for idx = component(:).'
            if idx == centerIdx || ismember(idx, fixedNodes)
                continue;
            end
            candidates = [];
            for k = 1:numel(edges)
                if edges(k).src == idx
                    candidates(end+1) = centers(edges(k).dst) + ...
                        dstOffset(k) - srcOffset(k); %#ok<AGROW>
                elseif edges(k).dst == idx
                    candidates(end+1) = centers(edges(k).src) + ...
                        srcOffset(k) - dstOffset(k); %#ok<AGROW>
                end
            end
            candidateValues = unique(round(candidates));
            for candidateIndex = 1:numel(candidateValues)
                candidate = candidateValues(candidateIndex);
                trial = centers;
                trial(idx) = candidate;
                gain = currentScore - score(trial, edges, ...
                    srcOffset, dstOffset, cfg.Tolerance);
                shift = candidate - centers(idx);
                rect = positions(idx, :);
                rect([2, 4]) = rect([2, 4]) + shift;
                if gain > bestGain && ~blocked(rect, idx, positions, cfg.Pad)
                    bestGain = gain;
                    bestIdx = idx;
                    bestCenter = candidate;
                end
            end
        end
        if bestIdx == 0
            break;
        end
        centers(bestIdx) = bestCenter;
        shift = bestCenter - (positions(bestIdx, 2) + positions(bestIdx, 4))/2;
        positions(bestIdx, [2, 4]) = positions(bestIdx, [2, 4]) + shift;
    end
    applyPositions(info, component, positions(component, :));
end

function value = score(centers, edges, srcOffset, dstOffset, tolerance)
    errors = zeros(1, numel(edges));
    for k = 1:numel(edges)
        errors(k) = abs(centers(edges(k).src) + srcOffset(k) - ...
            centers(edges(k).dst) - dstOffset(k));
    end
    value = 100*sum(errors > tolerance) + sum(errors);
end

function correctExactAlignment(info, component, edges, centerIdx, spine)
    positions = getPositions(info, 1:numel(info.blocks));
    centers = (positions(:, 2) + positions(:, 4))/2;
    for idx = component(:).'
        if idx == centerIdx || ismember(idx, spine)
            continue;
        end
        targets = [];
        for edge = edges(:).'
            if edge.line <= 0
                continue;
            end
            srcOffset = portY(edge.srcPort) - centers(edge.src);
            dstOffset = portY(edge.dstPort) - centers(edge.dst);
            if edge.src == idx
                targets(end+1) = centers(edge.dst) + ...
                    dstOffset - srcOffset; %#ok<AGROW>
            elseif edge.dst == idx
                targets(end+1) = centers(edge.src) + ...
                    srcOffset - dstOffset; %#ok<AGROW>
            end
        end
        if isempty(targets) || max(targets)-min(targets) > 1
            continue;
        end
        target = median(targets);
        shift = target - centers(idx);
        if abs(shift) < 0.5 || abs(shift) > 10
            continue;
        end
        trial = positions(idx, :);
        trial([2, 4]) = trial([2, 4]) + shift;
        if ~blocked(trial, idx, positions, 0)
            centers(idx) = target;
            positions(idx, [2, 4]) = positions(idx, [2, 4]) + shift;
            applyPositions(info, idx, positions(idx, :));
        end
    end
end

function alignToSpineGrid(info, component, edges, rank, spine)
    if numel(spine) < 2
        return;
    end
    positions = getPositions(info, 1:numel(info.blocks));
    xLeft = (positions(spine(1), 1)+positions(spine(1), 3))/2;
    xRight = (positions(spine(end), 1)+positions(spine(end), 3))/2;
    rankMin = min(rank(component));
    rankMax = max(rank(component));
    [inputs, outputs] = boundaryNodes(info, component, edges);
    for idx = component(:).'
        if ismember(idx, spine)
            continue;
        end
        if ismember(idx, inputs)
            xCenter = xLeft;
        elseif ismember(idx, outputs)
            xCenter = xRight;
        elseif rankMax > rankMin
            fraction = (rank(idx)-rankMin)/(rankMax-rankMin);
            xCenter = xLeft + fraction*(xRight-xLeft);
        else
            xCenter = xLeft;
        end
        width = positions(idx, 3)-positions(idx, 1);
        positions(idx, [1, 3]) = xCenter + [-width/2, width/2];
    end
    applyPositions(info, component, positions(component, :));
end

function [inputs, outputs] = boundaryNodes(info, component, edges)
    n = numel(info.blocks);
    indegree = zeros(n, 1);
    outdegree = zeros(n, 1);
    for edge = edges(:).'
        indegree(edge.dst) = indegree(edge.dst)+1;
        outdegree(edge.src) = outdegree(edge.src)+1;
    end
    inputs = component(indegree(component) == 0 & outdegree(component) > 0);
    outputs = component(outdegree(component) == 0 & indegree(component) > 0);
end

function arrangeBoundaryPorts(info, component, edges, spine, cfg)
    [inputs, outputs] = boundaryNodes(info, component, edges);
    positions = getPositions(info, 1:numel(info.blocks));
    xLeft = (positions(spine(1), 1)+positions(spine(1), 3))/2;
    xRight = (positions(spine(end), 1)+positions(spine(end), 3))/2;
    positions = arrangeBoundaryGroup(positions, inputs, edges, ...
        xLeft, 'in', spine, cfg);
    positions = arrangeBoundaryGroup(positions, outputs, edges, ...
        xRight, 'out', spine, cfg);
    boundary = unique([inputs(:); outputs(:)]);
    applyPositions(info, boundary, positions(boundary, :));
end

function positions = arrangeBoundaryGroup(positions, inds, edges, ...
        xCenter, direction, spine, cfg)
    if isempty(inds)
        return;
    end
    desired = zeros(1, numel(inds));
    for k = 1:numel(inds)
        values = [];
        for edge = edges(:).'
            if strcmp(direction, 'in') && edge.src == inds(k)
                values(end+1) = portY(edge.dstPort); %#ok<AGROW>
            elseif strcmp(direction, 'out') && edge.dst == inds(k)
                values(end+1) = portY(edge.srcPort); %#ok<AGROW>
            end
        end
        if isempty(values)
            desired(k) = (positions(inds(k), 2)+positions(inds(k), 4))/2;
        else
            desired(k) = median(values);
        end
    end
    heights = positions(inds, 4)-positions(inds, 2);
    fixed = find(ismember(inds, spine), 1);
    if isempty(fixed)
        centers = packClose(desired, heights, cfg.GapY);
    else
        fixedY = (positions(inds(fixed), 2)+positions(inds(fixed), 4))/2;
        centers = packAroundFixed(desired, heights, fixedY, fixed, cfg.GapY);
    end
    for k = 1:numel(inds)
        width = positions(inds(k), 3)-positions(inds(k), 1);
        positions(inds(k), :) = round([xCenter-width/2, ...
            centers(k)-heights(k)/2, xCenter+width/2, centers(k)+heights(k)/2]);
    end
end

function centers = packClose(desired, heights, gap)
    centers = desired;
    [~, order] = sort(desired);
    for k = 2:numel(order)
        prev = order(k-1);
        current = order(k);
        centers(current) = max(centers(current), ...
            centers(prev)+(heights(prev)+heights(current))/2+gap);
    end
    centers = centers + mean(desired-centers);
end

function centers = packAroundFixed(desired, heights, fixedY, fixed, gap)
    centers = desired;
    centers(fixed) = fixedY;
    others = setdiff(1:numel(desired), fixed);
    [~, order] = sort(desired(others));
    others = others(order);
    upper = others(desired(others) < fixedY);
    lower = others(desired(others) >= fixedY);
    boundary = fixedY-heights(fixed)/2-gap;
    for k = numel(upper):-1:1
        idx = upper(k);
        centers(idx) = min(desired(idx), boundary-heights(idx)/2);
        boundary = centers(idx)-heights(idx)/2-gap;
    end
    boundary = fixedY+heights(fixed)/2+gap;
    for k = 1:numel(lower)
        idx = lower(k);
        centers(idx) = max(desired(idx), boundary+heights(idx)/2);
        boundary = centers(idx)+heights(idx)/2+gap;
    end
end

function tf = blocked(rect, idx, positions, pad)
    tf = false;
    for k = 1:size(positions, 1)
        if k ~= idx && overlap(rect, positions(k, :), pad)
            tf = true;
            return;
        end
    end
end

function resolveOverlaps(info, component, rank, centerIdx, cfg)
    positions = getPositions(info, 1:numel(info.blocks));
    movable = ismember(1:numel(info.blocks), component);
    centerDistance = abs(rank - rank(centerIdx));
    for passIdx = 1:50
        changed = false;
        for i = 1:size(positions, 1)
            for j = i+1:size(positions, 1)
                if ~overlap(positions(i, :), positions(j, :), cfg.Pad)
                    continue;
                end
                if movable(j) && (~movable(i) || ...
                        (i ~= centerIdx && centerDistance(j) >= centerDistance(i)))
                    mover = j;
                    other = i;
                elseif movable(i)
                    mover = i;
                    other = j;
                else
                    continue;
                end
                a = positions(mover, :);
                b = positions(other, :);
                if (a(2)+a(4))/2 <= (b(2)+b(4))/2
                    delta = b(2) - cfg.Pad - a(4);
                else
                    delta = b(4) + cfg.Pad - a(2);
                end
                positions(mover, [2, 4]) = positions(mover, [2, 4]) + delta;
                applyPositions(info, mover, positions(mover, :));
                changed = true;
                break;
            end
            if changed
                break;
            end
        end
        if ~changed
            break;
        end
    end
end

function count = countOverlaps(info, component, pad)
    positions = getPositions(info, 1:numel(info.blocks));
    count = 0;
    for i = 1:size(positions, 1)
        for j = i+1:size(positions, 1)
            if (ismember(i, component) || ismember(j, component)) && ...
                    overlap(positions(i, :), positions(j, :), pad)
                count = count + 1;
            end
        end
    end
end

function tf = overlap(a, b, pad)
    tf = a(1) < b(3) + pad && b(1) < a(3) + pad && ...
        a(2) < b(4) + pad && b(2) < a(4) + pad;
end

function [maxError, aligned, total] = straightPortMetrics(edges, tolerance)
    maxError = 0;
    aligned = 0;
    total = 0;
    for e = edges(:).'
        if e.line <= 0
            continue;
        end
        error = abs(portY(e.srcPort) - portY(e.dstPort));
        maxError = max(maxError, error);
        aligned = aligned + (error <= tolerance);
        total = total + 1;
    end
end

function [diagonalCount, blockHits, badIndices] = lineGeometry(info)
    diagonalCount = 0;
    blockHits = 0;
    badIndices = [];
    positions = getPositions(info, 1:numel(info.blocks));
    for lineIdx = 1:numel(info.lines)
        lineH = info.lines(lineIdx);
        points = get_param(lineH, 'Points');
        hasDiagonal = false;
        if size(points, 1) > 1
            delta = diff(points, 1, 1);
            diagonalSegments = abs(delta(:, 1)) > 0.5 & ...
                abs(delta(:, 2)) > 0.5;
            hasDiagonal = any(diagonalSegments);
            diagonalCount = diagonalCount + sum(diagonalSegments);
        end
        srcPort = get_param(lineH, 'SrcPortHandle');
        dstPorts = get_param(lineH, 'DstPortHandle');
        ignore = portBlock(info, srcPort);
        for d = 1:numel(dstPorts)
            ignore(end+1) = portBlock(info, dstPorts(d)); %#ok<AGROW>
        end
        hitsBlock = lineHitsBlocks(points, positions, ignore);
        if hitsBlock
            blockHits = blockHits + 1;
        end
        if hasDiagonal || hitsBlock
            badIndices(end+1) = lineIdx; %#ok<AGROW>
        end
    end
end

function normalizeBranchTrunks(info, cfg)
    rects = getPositions(info, 1:numel(info.blocks));
    topY = min(rects(:, 2)) - cfg.GapY;
    bottomY = max(rects(:, 4)) + cfg.GapY;
    for lineH = info.lines(:).'
        dstPorts = get_param(lineH, 'DstPortHandle');
        if numel(dstPorts) < 2
            continue;
        end
        srcPort = get_param(lineH, 'SrcPortHandle');
        if srcPort <= 0
            continue;
        end
        srcPos = get_param(srcPort, 'Position');
        dstX = zeros(1, numel(dstPorts));
        for d = 1:numel(dstPorts)
            pos = get_param(dstPorts(d), 'Position');
            dstX(d) = pos(1);
        end
        if all(dstX > srcPos(1))
            branchX = min(dstX) - cfg.Pad;
        elseif all(dstX < srcPos(1))
            branchX = max(dstX) + cfg.Pad;
        else
            branchX = round((min(dstX)+max(dstX))/2);
        end
        ignore = portBlock(info, srcPort);
        for d = 1:numel(dstPorts)
            ignore(end+1) = portBlock(info, dstPorts(d)); %#ok<AGROW>
        end
        candidates = cell(1, 3);
        candidates{1} = round([srcPos; branchX, srcPos(2)]);
        candidates{2} = round([srcPos; srcPos(1), topY; branchX, topY]);
        candidates{3} = round([srcPos; srcPos(1), bottomY; branchX, bottomY]);
        best = candidates{1};
        for c = 1:numel(candidates)
            if ~lineHitsBlocks(candidates{c}, rects, ignore)
                best = candidates{c};
                break;
            end
        end
        try
            set_param(lineH, 'Points', best);
            drawnow;
        catch
        end
    end
end

function movedTotal = repairBlockCrossings(info, lines, cfg)
    movedTotal = 0;
    for passIdx = 1:3
        rects = getPositions(info, 1:numel(info.blocks));
        topY = min(rects(:, 2)) - cfg.GapY;
        bottomY = max(rects(:, 4)) + cfg.GapY;
        moved = false;
        for lineIdx = numel(lines):-1:1
            lineH = lines(lineIdx);
            dstPorts = get_param(lineH, 'DstPortHandle');
            if ~isscalar(dstPorts) || dstPorts(1) <= 0
                continue;
            end
            srcPort = get_param(lineH, 'SrcPortHandle');
            if srcPort <= 0
                continue;
            end
            dstPort = dstPorts(1);
            points = get_param(lineH, 'Points');
            srcPos = points(1, :);
            dstPos = get_param(dstPort, 'Position');
            ignore = [portBlock(info, srcPort), portBlock(info, dstPort)];
            dx = sign(dstPos(1) - srcPos(1));
            if dx == 0
                dx = 1;
            end
            stubX = srcPos(1) + dx*max(12, cfg.GapY);
            entryX = dstPos(1) - dx*max(12, cfg.GapY);
            candidates = cell(1, 3);
            candidates{1} = round([srcPos; stubX, srcPos(2); ...
                stubX, topY; entryX, topY; entryX, dstPos(2); dstPos]);
            candidates{2} = round([srcPos; stubX, srcPos(2); ...
                stubX, bottomY; entryX, bottomY; entryX, dstPos(2); dstPos]);
            midX = round((srcPos(1)+dstPos(1))/2);
            candidates{3} = round([srcPos; midX, srcPos(2); ...
                midX, dstPos(2); dstPos]);
            best = [];
            for c = 1:numel(candidates)
                if ~lineHitsBlocks(candidates{c}, rects, ignore) && ...
                        (isempty(best) || pathLength(candidates{c}) < ...
                        pathLength(best))
                    best = candidates{c};
                end
            end
            if ~isempty(best)
                try
                    set_param(lineH, 'Points', best);
                    drawnow;
                    moved = true;
                    movedTotal = movedTotal + 1;
                catch
                end
            end
        end
        if ~moved
            break;
        end
    end
end

function value = pathLength(points)
    delta = diff(points, 1, 1);
    value = sum(sqrt(sum(delta.^2, 2)));
end

function idx = portBlock(info, port)
    idx = [];
    if port > 0 && isKey(info.blockMap, get_param(port, 'Parent'))
        idx = info.blockMap(get_param(port, 'Parent'));
    end
end

function hit = lineHitsBlocks(points, rects, ignore)
    hit = false;
    for s = 1:size(points, 1)-1
        samples = max(2, ceil(max(abs(points(s+1, :)-points(s, :)))/2));
        for t = linspace(0, 1, samples)
            p = round(points(s, :) + t*(points(s+1, :)-points(s, :)));
            for k = 1:size(rects, 1)
                if ~ismember(k, ignore) && p(1) > rects(k, 1)+1 && ...
                        p(1) < rects(k, 3)-1 && p(2) > rects(k, 2)+1 && ...
                        p(2) < rects(k, 4)-1
                    hit = true;
                    return;
                end
            end
        end
    end
end

function y = portY(port)
    y = get_param(port, 'Position');
    y = y(2);
end

function positions = getPositions(info, indices)
    positions = zeros(numel(indices), 4);
    for k = 1:numel(indices)
        positions(k, :) = get_param(info.blocks{indices(k)}, 'Position');
    end
end

function applyPositions(info, indices, positions)
    for k = 1:numel(indices)
        set_param(info.blocks{indices(k)}, 'Position', round(positions(k, :)));
    end
end

function routeLines(lines)
    try
        Simulink.BlockDiagram.routeLine(lines(lines > 0));
        drawnow;
        pause(0.05);
        drawnow;
    catch
    end
end
