clear

load auction_input.mat
disp('Input Cost')
disp(costMatrix)

[rowSoln, colSoln, colRedux] = auctionTest(costMatrix, varargin);


%disp(rowSoln)
disp('resulting assignment')
disp(colSoln)
disp(colRedux)