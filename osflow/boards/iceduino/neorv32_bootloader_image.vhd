-- The NEORV32 RISC-V Processor, https://github.com/stnolting/neorv32
-- Auto-generated memory init file (for BOOTLOADER) from source file <bootloader/main.bin>
-- Size: 2284 bytes

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

package neorv32_bootloader_image is

  constant bootloader_init_image : mem32_t := (
    00000000 => x"00000037",
    00000001 => x"80010117",
    00000002 => x"1f810113",
    00000003 => x"80010197",
    00000004 => x"7f418193",
    00000005 => x"00000517",
    00000006 => x"11850513",
    00000007 => x"30551073",
    00000008 => x"34151073",
    00000009 => x"30001073",
    00000010 => x"30401073",
    00000011 => x"34401073",
    00000012 => x"32001073",
    00000013 => x"30601073",
    00000014 => x"b0001073",
    00000015 => x"b8001073",
    00000016 => x"b0201073",
    00000017 => x"b8201073",
    00000018 => x"00000093",
    00000019 => x"00000213",
    00000020 => x"00000293",
    00000021 => x"00000313",
    00000022 => x"00000393",
    00000023 => x"00000813",
    00000024 => x"00000893",
    00000025 => x"00000913",
    00000026 => x"00000993",
    00000027 => x"00000a13",
    00000028 => x"00000a93",
    00000029 => x"00000b13",
    00000030 => x"00000b93",
    00000031 => x"00000c13",
    00000032 => x"00000c93",
    00000033 => x"00000d13",
    00000034 => x"00000d93",
    00000035 => x"00000e13",
    00000036 => x"00000e93",
    00000037 => x"00000f13",
    00000038 => x"00000f93",
    00000039 => x"00010417",
    00000040 => x"d6440413",
    00000041 => x"00010497",
    00000042 => x"f5c48493",
    00000043 => x"00042023",
    00000044 => x"00440413",
    00000045 => x"fe941ce3",
    00000046 => x"00001597",
    00000047 => x"83458593",
    00000048 => x"80010617",
    00000049 => x"f4060613",
    00000050 => x"80010697",
    00000051 => x"f3868693",
    00000052 => x"00d65c63",
    00000053 => x"00058703",
    00000054 => x"00e60023",
    00000055 => x"00158593",
    00000056 => x"00160613",
    00000057 => x"fedff06f",
    00000058 => x"80010717",
    00000059 => x"f1870713",
    00000060 => x"80818793",
    00000061 => x"00f75863",
    00000062 => x"00070023",
    00000063 => x"00170713",
    00000064 => x"ff5ff06f",
    00000065 => x"00000513",
    00000066 => x"00000593",
    00000067 => x"06c000ef",
    00000068 => x"34051073",
    00000069 => x"00000093",
    00000070 => x"00008463",
    00000071 => x"000080e7",
    00000072 => x"30047073",
    00000073 => x"10500073",
    00000074 => x"0000006f",
    00000075 => x"ff810113",
    00000076 => x"00812023",
    00000077 => x"00912223",
    00000078 => x"34202473",
    00000079 => x"02044663",
    00000080 => x"34102473",
    00000081 => x"00041483",
    00000082 => x"0034f493",
    00000083 => x"00240413",
    00000084 => x"34141073",
    00000085 => x"00300413",
    00000086 => x"00941863",
    00000087 => x"34102473",
    00000088 => x"00240413",
    00000089 => x"34141073",
    00000090 => x"00012403",
    00000091 => x"00412483",
    00000092 => x"00810113",
    00000093 => x"30200073",
    00000094 => x"00005537",
    00000095 => x"ff010113",
    00000096 => x"00000613",
    00000097 => x"00000593",
    00000098 => x"b0050513",
    00000099 => x"00112623",
    00000100 => x"3cc000ef",
    00000101 => x"00000693",
    00000102 => x"00000613",
    00000103 => x"00000593",
    00000104 => x"00200513",
    00000105 => x"500000ef",
    00000106 => x"ffff1537",
    00000107 => x"84050513",
    00000108 => x"49c000ef",
    00000109 => x"00030537",
    00000110 => x"0b0000ef",
    00000111 => x"ffff1537",
    00000112 => x"87050513",
    00000113 => x"488000ef",
    00000114 => x"00100513",
    00000115 => x"250000ef",
    00000116 => x"ffff1537",
    00000117 => x"87850513",
    00000118 => x"474000ef",
    00000119 => x"004000ef",
    00000120 => x"ff010113",
    00000121 => x"00112623",
    00000122 => x"30047073",
    00000123 => x"ffff0537",
    00000124 => x"7f050513",
    00000125 => x"458000ef",
    00000126 => x"428000ef",
    00000127 => x"fe051ee3",
    00000128 => x"ff002783",
    00000129 => x"00078067",
    00000130 => x"0000006f",
    00000131 => x"ff010113",
    00000132 => x"00812423",
    00000133 => x"00050413",
    00000134 => x"ffff1537",
    00000135 => x"80050513",
    00000136 => x"00112623",
    00000137 => x"428000ef",
    00000138 => x"03040513",
    00000139 => x"0ff57513",
    00000140 => x"3d8000ef",
    00000141 => x"03a00513",
    00000142 => x"3d0000ef",
    00000143 => x"02000513",
    00000144 => x"3c8000ef",
    00000145 => x"00141793",
    00000146 => x"008787b3",
    00000147 => x"ffff1537",
    00000148 => x"00379793",
    00000149 => x"87c50513",
    00000150 => x"00f50533",
    00000151 => x"3f0000ef",
    00000152 => x"30047073",
    00000153 => x"0000006f",
    00000154 => x"fe010113",
    00000155 => x"01212823",
    00000156 => x"00050913",
    00000157 => x"ffff1537",
    00000158 => x"00912a23",
    00000159 => x"80c50513",
    00000160 => x"ffff14b7",
    00000161 => x"00812c23",
    00000162 => x"01312623",
    00000163 => x"00112e23",
    00000164 => x"01c00413",
    00000165 => x"3b8000ef",
    00000166 => x"8dc48493",
    00000167 => x"ffc00993",
    00000168 => x"008957b3",
    00000169 => x"00f7f793",
    00000170 => x"00f487b3",
    00000171 => x"0007c503",
    00000172 => x"ffc40413",
    00000173 => x"354000ef",
    00000174 => x"ff3414e3",
    00000175 => x"01c12083",
    00000176 => x"01812403",
    00000177 => x"01412483",
    00000178 => x"01012903",
    00000179 => x"00c12983",
    00000180 => x"02010113",
    00000181 => x"00008067",
    00000182 => x"ff010113",
    00000183 => x"00000513",
    00000184 => x"00112623",
    00000185 => x"00812423",
    00000186 => x"3fc000ef",
    00000187 => x"09000513",
    00000188 => x"438000ef",
    00000189 => x"00000513",
    00000190 => x"430000ef",
    00000191 => x"00050413",
    00000192 => x"00000513",
    00000193 => x"400000ef",
    00000194 => x"00c12083",
    00000195 => x"0ff47513",
    00000196 => x"00812403",
    00000197 => x"01010113",
    00000198 => x"00008067",
    00000199 => x"ff010113",
    00000200 => x"00812423",
    00000201 => x"00050413",
    00000202 => x"01055513",
    00000203 => x"0ff57513",
    00000204 => x"00112623",
    00000205 => x"3f4000ef",
    00000206 => x"00845513",
    00000207 => x"0ff57513",
    00000208 => x"3e8000ef",
    00000209 => x"0ff47513",
    00000210 => x"00812403",
    00000211 => x"00c12083",
    00000212 => x"01010113",
    00000213 => x"3d40006f",
    00000214 => x"ff010113",
    00000215 => x"00812423",
    00000216 => x"00050413",
    00000217 => x"00000513",
    00000218 => x"00112623",
    00000219 => x"378000ef",
    00000220 => x"00300513",
    00000221 => x"3b4000ef",
    00000222 => x"00040513",
    00000223 => x"fa1ff0ef",
    00000224 => x"00000513",
    00000225 => x"3a4000ef",
    00000226 => x"00050413",
    00000227 => x"00000513",
    00000228 => x"374000ef",
    00000229 => x"00c12083",
    00000230 => x"0ff47513",
    00000231 => x"00812403",
    00000232 => x"01010113",
    00000233 => x"00008067",
    00000234 => x"fd010113",
    00000235 => x"02812423",
    00000236 => x"02912223",
    00000237 => x"03212023",
    00000238 => x"01312e23",
    00000239 => x"02112623",
    00000240 => x"00050993",
    00000241 => x"00058493",
    00000242 => x"00c10913",
    00000243 => x"00358413",
    00000244 => x"04099063",
    00000245 => x"268000ef",
    00000246 => x"00a90023",
    00000247 => x"fff40793",
    00000248 => x"00190913",
    00000249 => x"02849263",
    00000250 => x"02c12083",
    00000251 => x"02812403",
    00000252 => x"00c12503",
    00000253 => x"02412483",
    00000254 => x"02012903",
    00000255 => x"01c12983",
    00000256 => x"03010113",
    00000257 => x"00008067",
    00000258 => x"00078413",
    00000259 => x"fc5ff06f",
    00000260 => x"00040513",
    00000261 => x"f45ff0ef",
    00000262 => x"fc1ff06f",
    00000263 => x"fd010113",
    00000264 => x"01412c23",
    00000265 => x"80000a37",
    00000266 => x"02812423",
    00000267 => x"004a0793",
    00000268 => x"02112623",
    00000269 => x"02912223",
    00000270 => x"03212023",
    00000271 => x"01312e23",
    00000272 => x"01512a23",
    00000273 => x"01612823",
    00000274 => x"01712623",
    00000275 => x"01812423",
    00000276 => x"00100713",
    00000277 => x"00e7a023",
    00000278 => x"00050413",
    00000279 => x"004a0a13",
    00000280 => x"02051863",
    00000281 => x"ffff1537",
    00000282 => x"81050513",
    00000283 => x"1e0000ef",
    00000284 => x"000305b7",
    00000285 => x"00040513",
    00000286 => x"f31ff0ef",
    00000287 => x"4788d7b7",
    00000288 => x"afe78793",
    00000289 => x"02f50463",
    00000290 => x"00000513",
    00000291 => x"01c0006f",
    00000292 => x"ffff1537",
    00000293 => x"83050513",
    00000294 => x"1b4000ef",
    00000295 => x"e3dff0ef",
    00000296 => x"fc0518e3",
    00000297 => x"00300513",
    00000298 => x"d65ff0ef",
    00000299 => x"000309b7",
    00000300 => x"00498593",
    00000301 => x"00040513",
    00000302 => x"ef1ff0ef",
    00000303 => x"00050a93",
    00000304 => x"00898593",
    00000305 => x"00040513",
    00000306 => x"ee1ff0ef",
    00000307 => x"ff002c03",
    00000308 => x"00050b13",
    00000309 => x"ffcafb93",
    00000310 => x"00000913",
    00000311 => x"00000493",
    00000312 => x"00c98993",
    00000313 => x"013905b3",
    00000314 => x"052b9c63",
    00000315 => x"016484b3",
    00000316 => x"00200513",
    00000317 => x"fa049ae3",
    00000318 => x"ffff1537",
    00000319 => x"83c50513",
    00000320 => x"14c000ef",
    00000321 => x"02c12083",
    00000322 => x"02812403",
    00000323 => x"800007b7",
    00000324 => x"0157a023",
    00000325 => x"000a2023",
    00000326 => x"02412483",
    00000327 => x"02012903",
    00000328 => x"01c12983",
    00000329 => x"01812a03",
    00000330 => x"01412a83",
    00000331 => x"01012b03",
    00000332 => x"00c12b83",
    00000333 => x"00812c03",
    00000334 => x"03010113",
    00000335 => x"00008067",
    00000336 => x"00040513",
    00000337 => x"e65ff0ef",
    00000338 => x"012c07b3",
    00000339 => x"00a484b3",
    00000340 => x"00a7a023",
    00000341 => x"00490913",
    00000342 => x"f8dff06f",
    00000343 => x"ff010113",
    00000344 => x"00812423",
    00000345 => x"00912223",
    00000346 => x"00112623",
    00000347 => x"fa002023",
    00000348 => x"fe002783",
    00000349 => x"00058413",
    00000350 => x"00151593",
    00000351 => x"00078513",
    00000352 => x"00060493",
    00000353 => x"1c0000ef",
    00000354 => x"01051513",
    00000355 => x"000017b7",
    00000356 => x"01055513",
    00000357 => x"00000713",
    00000358 => x"ffe78793",
    00000359 => x"04a7e463",
    00000360 => x"0034f793",
    00000361 => x"00347413",
    00000362 => x"fff50513",
    00000363 => x"01479793",
    00000364 => x"01641413",
    00000365 => x"00f567b3",
    00000366 => x"0087e7b3",
    00000367 => x"01871713",
    00000368 => x"00c12083",
    00000369 => x"00812403",
    00000370 => x"00e7e7b3",
    00000371 => x"10000737",
    00000372 => x"00e7e7b3",
    00000373 => x"faf02023",
    00000374 => x"00412483",
    00000375 => x"01010113",
    00000376 => x"00008067",
    00000377 => x"ffe70693",
    00000378 => x"0fd6f693",
    00000379 => x"00069a63",
    00000380 => x"00355513",
    00000381 => x"00170713",
    00000382 => x"0ff77713",
    00000383 => x"fa1ff06f",
    00000384 => x"00155513",
    00000385 => x"ff1ff06f",
    00000386 => x"00040737",
    00000387 => x"fa002783",
    00000388 => x"00e7f7b3",
    00000389 => x"fe079ce3",
    00000390 => x"faa02223",
    00000391 => x"00008067",
    00000392 => x"fa002783",
    00000393 => x"00100513",
    00000394 => x"0007c863",
    00000395 => x"0107d513",
    00000396 => x"00154513",
    00000397 => x"00157513",
    00000398 => x"00008067",
    00000399 => x"fa402503",
    00000400 => x"fe055ee3",
    00000401 => x"0ff57513",
    00000402 => x"00008067",
    00000403 => x"ff010113",
    00000404 => x"00812423",
    00000405 => x"01212023",
    00000406 => x"00112623",
    00000407 => x"00912223",
    00000408 => x"00050413",
    00000409 => x"00a00913",
    00000410 => x"00044483",
    00000411 => x"00140413",
    00000412 => x"00049e63",
    00000413 => x"00c12083",
    00000414 => x"00812403",
    00000415 => x"00412483",
    00000416 => x"00012903",
    00000417 => x"01010113",
    00000418 => x"00008067",
    00000419 => x"01249663",
    00000420 => x"00d00513",
    00000421 => x"f75ff0ef",
    00000422 => x"00048513",
    00000423 => x"f6dff0ef",
    00000424 => x"fc9ff06f",
    00000425 => x"00757513",
    00000426 => x"0036f793",
    00000427 => x"00167613",
    00000428 => x"00a51513",
    00000429 => x"00d79793",
    00000430 => x"0015f593",
    00000431 => x"00f567b3",
    00000432 => x"00f61613",
    00000433 => x"00c7e7b3",
    00000434 => x"00959593",
    00000435 => x"fa800713",
    00000436 => x"00b7e7b3",
    00000437 => x"00072023",
    00000438 => x"1007e793",
    00000439 => x"00f72023",
    00000440 => x"00008067",
    00000441 => x"fa800713",
    00000442 => x"00072683",
    00000443 => x"00757793",
    00000444 => x"00100513",
    00000445 => x"00f51533",
    00000446 => x"00d56533",
    00000447 => x"00a72023",
    00000448 => x"00008067",
    00000449 => x"fa800713",
    00000450 => x"00072683",
    00000451 => x"00757513",
    00000452 => x"00100793",
    00000453 => x"00a797b3",
    00000454 => x"fff7c793",
    00000455 => x"00d7f7b3",
    00000456 => x"00f72023",
    00000457 => x"00008067",
    00000458 => x"faa02623",
    00000459 => x"fa802783",
    00000460 => x"fe07cee3",
    00000461 => x"fac02503",
    00000462 => x"00008067",
    00000463 => x"06054063",
    00000464 => x"0605c663",
    00000465 => x"00058613",
    00000466 => x"00050593",
    00000467 => x"fff00513",
    00000468 => x"02060c63",
    00000469 => x"00100693",
    00000470 => x"00b67a63",
    00000471 => x"00c05863",
    00000472 => x"00161613",
    00000473 => x"00169693",
    00000474 => x"feb66ae3",
    00000475 => x"00000513",
    00000476 => x"00c5e663",
    00000477 => x"40c585b3",
    00000478 => x"00d56533",
    00000479 => x"0016d693",
    00000480 => x"00165613",
    00000481 => x"fe0696e3",
    00000482 => x"00008067",
    00000483 => x"00008293",
    00000484 => x"fb5ff0ef",
    00000485 => x"00058513",
    00000486 => x"00028067",
    00000487 => x"40a00533",
    00000488 => x"00b04863",
    00000489 => x"40b005b3",
    00000490 => x"f9dff06f",
    00000491 => x"40b005b3",
    00000492 => x"00008293",
    00000493 => x"f91ff0ef",
    00000494 => x"40a00533",
    00000495 => x"00028067",
    00000496 => x"00008293",
    00000497 => x"0005ca63",
    00000498 => x"00054c63",
    00000499 => x"f79ff0ef",
    00000500 => x"00058513",
    00000501 => x"00028067",
    00000502 => x"40b005b3",
    00000503 => x"fe0558e3",
    00000504 => x"40a00533",
    00000505 => x"f61ff0ef",
    00000506 => x"40b00533",
    00000507 => x"00028067",
    00000508 => x"746f6f42",
    00000509 => x"2e676e69",
    00000510 => x"0a0a2e2e",
    00000511 => x"00000000",
    00000512 => x"52450a07",
    00000513 => x"5f524f52",
    00000514 => x"00000000",
    00000515 => x"00007830",
    00000516 => x"69617741",
    00000517 => x"676e6974",
    00000518 => x"6f656e20",
    00000519 => x"32337672",
    00000520 => x"6578655f",
    00000521 => x"6e69622e",
    00000522 => x"202e2e2e",
    00000523 => x"00000000",
    00000524 => x"64616f4c",
    00000525 => x"2e676e69",
    00000526 => x"00202e2e",
    00000527 => x"00004b4f",
    00000528 => x"4f454e0a",
    00000529 => x"32335652",
    00000530 => x"6f6f6220",
    00000531 => x"616f6c74",
    00000532 => x"0a726564",
    00000533 => x"64616f4c",
    00000534 => x"20676e69",
    00000535 => x"6d6f7266",
    00000536 => x"49505320",
    00000537 => x"616c6620",
    00000538 => x"61206873",
    00000539 => x"00002074",
    00000540 => x"0a2e2e2e",
    00000541 => x"00000000",
    00000542 => x"0000000a",
    00000543 => x"20657865",
    00000544 => x"6e676973",
    00000545 => x"72757461",
    00000546 => x"61662065",
    00000547 => x"00006c69",
    00000548 => x"00000000",
    00000549 => x"65637865",
    00000550 => x"6e696465",
    00000551 => x"4d492067",
    00000552 => x"63204d45",
    00000553 => x"63617061",
    00000554 => x"00797469",
    00000555 => x"63656863",
    00000556 => x"6d75736b",
    00000557 => x"69616620",
    00000558 => x"0000006c",
    00000559 => x"00000000",
    00000560 => x"00000000",
    00000561 => x"20495053",
    00000562 => x"73616c66",
    00000563 => x"63612068",
    00000564 => x"73736563",
    00000565 => x"69616620",
    00000566 => x"0064656c",
    00000567 => x"33323130",
    00000568 => x"37363534",
    00000569 => x"62613938",
    00000570 => x"66656463"
  );

end neorv32_bootloader_image;
