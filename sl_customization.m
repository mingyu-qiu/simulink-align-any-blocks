function sl_customization(cm)
    % 向 Simulink 的工具菜单注册自定义菜单项
    cm.addCustomMenuFcn('Simulink:ToolsMenu', @getMyMenuItems);
end

function schemaFcns = getMyMenuItems(~)
    schemaFcns = {@alignBlocksSchema};
end

function schema = alignBlocksSchema(~)
    schema = sl_action_schema;
    schema.label = 'Align Blocks';          % 菜单显示的名称
    schema.statustip = 'Run alignAnyBlocks'; % 鼠标悬停时的提示
    schema.accelerator = 'Ctrl+Shift+Q';    % 快捷键设置
    schema.callback = @alignBlocksCallback; % 绑定的回调函数
end

function alignBlocksCallback(~)
    % 调用你之前写好的对齐函数
    alignAnyBlocks();
end