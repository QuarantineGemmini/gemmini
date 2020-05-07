
import yaml 

print('loading output-referred')
outr = yaml.load(open('output_im2col.yml', 'r'))
print('loading input-referred')
inpr = yaml.load(open('input_im2col.yml', 'r'))

ind = {(a,b) : (c,d,e,f) for (a,b,c,d,e,f) in inpr}
opd = {(a,b) : (c,d,e,f) for (a,b,c,d,e,f) in outr}

for k in opd:
    if opd[k] == (-1,-1,-1,-1):
        assert ind.get(k) is None
    else:
        assert opd[k] == ind[k]

# ncols >= k*k*n_channels

