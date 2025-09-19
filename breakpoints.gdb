b ncr710_execute_script
b ncr710_do_dma
b ncr710_reg_writeb if offset == 0x2f
b ncr710_memcpy
