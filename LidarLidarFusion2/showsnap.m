function showsnap(snaps, id)
h = figure("Name","Snap #" + id,"IntegerHandle","off","Units","normalized","Position",[0.01 0.01 0.98 0.98]);
panels = snaps{id};
for i = 1:numel(panels)
    copyobj(panels{i},h);
end
end