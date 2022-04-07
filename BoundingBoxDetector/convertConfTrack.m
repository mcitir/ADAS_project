function convertedTrackList = convertConfTrack(trackList)
%convertConfTrack To convert Struct to matrix for X,Y,Z position
%   This function works for converting struct State data...
for i=1:numel(trackList)
    convertedTrackList(i,1)=trackList(i).State(1); %#ok<*AGROW> 
    convertedTrackList(i,2)=trackList(i).State(3);
    convertedTrackList(i,3)=trackList(i).State(6);
end

end