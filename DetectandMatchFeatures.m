function indexPairs = DetectandMatchFeatures(preFeatures,currFeatures)

[Lia,Locb]=ismember(preFeatures',currFeatures','rows');
indexPairs = zeros(sum(Lia),2); k = 0;
for i = 1:length(Lia)
   if Lia(i)
       k = k+1;
       indexPairs(k,:) = [i Locb(i)];
   end
end

end