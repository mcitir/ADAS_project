function snap = takesnap(f)
panels = findall(f,'Type','uipanel');
snap = {copy(panels)};
end
