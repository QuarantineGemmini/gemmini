"""
Run a few workloads, and write some YAML data of their matrix sizes
"""
import os 

testcases = {
  'template': '/chipyard/generators/gemmini/software/gemmini-rocc-tests/build_hw_tiler/bareMetalC/template-pk',
  'cifar': '/chipyard/generators/gemmini/software/gemmini-rocc-tests/build_hw_tiler/ee290/cifar_quant-pk',
  'mobilenet': '/chipyard/generators/gemmini/software/gemmini-rocc-tests/build_hw_tiler/imagenet/mobilenet-pk', 
  'resnet50': '/chipyard/generators/gemmini/software/gemmini-rocc-tests/build_hw_tiler/imagenet/resnet50-pk',
}

resultfile = 'sizes.yml'
with open(resultfile, 'w') as results:
  results.write('testcases:\n')

for tname in testcases:
  targ = testcases[tname]
  with open(resultfile, 'a') as results:
    results.write(f'  name: {tname} \n')
    results.write(f'  sizes: \n')
  
  # Yeah OK it's not much of a python script here is it 
  log_pattern = 'GEMMINI_SIZES:'
  cmd = f'spike --extension=gemmini2 pk {targ} | spike-dasm | grep "{log_pattern}" | sed "s/{log_pattern}//" | tee -a {resultfile}'
  print(cmd)
  os.system(cmd)


