clear all;

name = 'canvas6.ply';

[X, Y, Z] = sphere;

output = fopen(name, 'w');
fprintf(output, 'ply\n');
fprintf(output, 'format ascii 1.0\n');

rows = size (x,1);
columns = size (x,2);

facets = [];
index = reshape(0:numel(x)-1, columns, rows).';

for i = 1:rows - 1
    for j = 1:columns -1

        idx1 = index(i,j);
        idx2 = index(i,j+1);
        idx3 = index(i+1, j+1);

        facets = [facets; idx1 idx2 idx3];

        idx1 = index(i + 1,j + 1);
        idx2 = index(i+1,j);
        idx3 = index(i, j);

        facets = [facets;idx1 idx2 idx3];
    end
end

vx = X';
vy = Y';
vz = Z';
vertices = [vx(:) vy(:) vz(:)];

num_vertex = size(vertices,1);
num_facets = size(facets,1);

RGB = zeros(num_facets, 3);
colour =  255 * ones(1,num_facets)';
RGB(:,1) = colour;

fprintf(output, ['element vertex ' num2str(num_vertex)'\n']);
fprintf(output, 'property float x\nproperty float y\nproperty float z\n');
fprintf(output, ['element face ' num2str(num_facets)'\n']);
fprintf(output, 'property list uchar int vertex_indices\n');

for i = 1:size(vertices,1)
    fprintf(output, '%.17g %.17g %.17g ', vertices(i,:));

end

for i = 1:size(facets,1)
    fprintf(output, '3 %d %d %d\n', facets(i,:));
end

fclose(output)

disp([num2str(num_facets) ' facets']);
