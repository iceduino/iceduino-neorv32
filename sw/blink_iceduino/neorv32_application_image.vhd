-- The NEORV32 RISC-V Processor, https://github.com/stnolting/neorv32
-- Auto-generated memory init file (for APPLICATION) from source file <blink_iceduino/main.bin>
-- Size: 844 bytes

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

package neorv32_application_image is

  constant application_init_image : mem32_t := (
    00000000 => x"00000037",
    00000001 => x"80000117",
    00000002 => x"7f810113",
    00000003 => x"80000197",
    00000004 => x"7f418193",
    00000005 => x"00000517",
    00000006 => x"0d850513",
    00000007 => x"30551073",
    00000008 => x"34151073",
    00000009 => x"30001073",
    00000010 => x"30401073",
    00000011 => x"30601073",
    00000012 => x"ffa00593",
    00000013 => x"32059073",
    00000014 => x"b0001073",
    00000015 => x"b8001073",
    00000016 => x"b0201073",
    00000017 => x"b8201073",
    00000018 => x"00000093",
    00000019 => x"00000213",
    00000020 => x"00000293",
    00000021 => x"00000313",
    00000022 => x"00000393",
    00000023 => x"00000713",
    00000024 => x"00000793",
    00000025 => x"00000417",
    00000026 => x"d9c40413",
    00000027 => x"00000497",
    00000028 => x"f9448493",
    00000029 => x"00042023",
    00000030 => x"00440413",
    00000031 => x"fe941ce3",
    00000032 => x"80000597",
    00000033 => x"f8058593",
    00000034 => x"80000617",
    00000035 => x"f7860613",
    00000036 => x"00c5d863",
    00000037 => x"00058023",
    00000038 => x"00158593",
    00000039 => x"ff5ff06f",
    00000040 => x"00000597",
    00000041 => x"2ac58593",
    00000042 => x"80000617",
    00000043 => x"f5860613",
    00000044 => x"80000697",
    00000045 => x"f5068693",
    00000046 => x"00d65c63",
    00000047 => x"00058703",
    00000048 => x"00e60023",
    00000049 => x"00158593",
    00000050 => x"00160613",
    00000051 => x"fedff06f",
    00000052 => x"00000513",
    00000053 => x"00000593",
    00000054 => x"060000ef",
    00000055 => x"34051073",
    00000056 => x"30047073",
    00000057 => x"10500073",
    00000058 => x"ffdff06f",
    00000059 => x"ff810113",
    00000060 => x"00812023",
    00000061 => x"00912223",
    00000062 => x"34202473",
    00000063 => x"02044663",
    00000064 => x"34102473",
    00000065 => x"00041483",
    00000066 => x"0034f493",
    00000067 => x"00240413",
    00000068 => x"34141073",
    00000069 => x"00300413",
    00000070 => x"00941863",
    00000071 => x"34102473",
    00000072 => x"00240413",
    00000073 => x"34141073",
    00000074 => x"00012403",
    00000075 => x"00412483",
    00000076 => x"00810113",
    00000077 => x"30200073",
    00000078 => x"ff010113",
    00000079 => x"00812423",
    00000080 => x"00112623",
    00000081 => x"00000413",
    00000082 => x"028000ef",
    00000083 => x"0ff47513",
    00000084 => x"014000ef",
    00000085 => x"06400513",
    00000086 => x"024000ef",
    00000087 => x"00140413",
    00000088 => x"fedff06f",
    00000089 => x"a00007b7",
    00000090 => x"00a7a223",
    00000091 => x"00008067",
    00000092 => x"a00007b7",
    00000093 => x"0007a223",
    00000094 => x"00008067",
    00000095 => x"fe010113",
    00000096 => x"00112e23",
    00000097 => x"00050613",
    00000098 => x"00055863",
    00000099 => x"40a00633",
    00000100 => x"01061613",
    00000101 => x"41065613",
    00000102 => x"fe002503",
    00000103 => x"3e800593",
    00000104 => x"00c12623",
    00000105 => x"0fc000ef",
    00000106 => x"00c12603",
    00000107 => x"00000593",
    00000108 => x"41f65693",
    00000109 => x"054000ef",
    00000110 => x"01c59593",
    00000111 => x"00455513",
    00000112 => x"00a5e533",
    00000113 => x"00050a63",
    00000114 => x"00050863",
    00000115 => x"fff50513",
    00000116 => x"00000013",
    00000117 => x"ff1ff06f",
    00000118 => x"01c12083",
    00000119 => x"02010113",
    00000120 => x"00008067",
    00000121 => x"00050613",
    00000122 => x"00000513",
    00000123 => x"0015f693",
    00000124 => x"00068463",
    00000125 => x"00c50533",
    00000126 => x"0015d593",
    00000127 => x"00161613",
    00000128 => x"fe0596e3",
    00000129 => x"00008067",
    00000130 => x"00050313",
    00000131 => x"ff010113",
    00000132 => x"00060513",
    00000133 => x"00068893",
    00000134 => x"00112623",
    00000135 => x"00030613",
    00000136 => x"00050693",
    00000137 => x"00000713",
    00000138 => x"00000793",
    00000139 => x"00000813",
    00000140 => x"0016fe13",
    00000141 => x"00171e93",
    00000142 => x"000e0c63",
    00000143 => x"01060e33",
    00000144 => x"010e3833",
    00000145 => x"00e787b3",
    00000146 => x"00f807b3",
    00000147 => x"000e0813",
    00000148 => x"01f65713",
    00000149 => x"0016d693",
    00000150 => x"00eee733",
    00000151 => x"00161613",
    00000152 => x"fc0698e3",
    00000153 => x"00058663",
    00000154 => x"f7dff0ef",
    00000155 => x"00a787b3",
    00000156 => x"00088a63",
    00000157 => x"00030513",
    00000158 => x"00088593",
    00000159 => x"f69ff0ef",
    00000160 => x"00f507b3",
    00000161 => x"00c12083",
    00000162 => x"00080513",
    00000163 => x"00078593",
    00000164 => x"01010113",
    00000165 => x"00008067",
    00000166 => x"06054063",
    00000167 => x"0605c663",
    00000168 => x"00058613",
    00000169 => x"00050593",
    00000170 => x"fff00513",
    00000171 => x"02060c63",
    00000172 => x"00100693",
    00000173 => x"00b67a63",
    00000174 => x"00c05863",
    00000175 => x"00161613",
    00000176 => x"00169693",
    00000177 => x"feb66ae3",
    00000178 => x"00000513",
    00000179 => x"00c5e663",
    00000180 => x"40c585b3",
    00000181 => x"00d56533",
    00000182 => x"0016d693",
    00000183 => x"00165613",
    00000184 => x"fe0696e3",
    00000185 => x"00008067",
    00000186 => x"00008293",
    00000187 => x"fb5ff0ef",
    00000188 => x"00058513",
    00000189 => x"00028067",
    00000190 => x"40a00533",
    00000191 => x"00b04863",
    00000192 => x"40b005b3",
    00000193 => x"f9dff06f",
    00000194 => x"40b005b3",
    00000195 => x"00008293",
    00000196 => x"f91ff0ef",
    00000197 => x"40a00533",
    00000198 => x"00028067",
    00000199 => x"00008293",
    00000200 => x"0005ca63",
    00000201 => x"00054c63",
    00000202 => x"f79ff0ef",
    00000203 => x"00058513",
    00000204 => x"00028067",
    00000205 => x"40b005b3",
    00000206 => x"fe0558e3",
    00000207 => x"40a00533",
    00000208 => x"f61ff0ef",
    00000209 => x"40b00533",
    00000210 => x"00028067"
  );

end neorv32_application_image;