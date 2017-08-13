/**
 * Copyright 2011 B. Schauerte. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are 
 * met:
 * 
 *    1. Redistributions of source code must retain the above copyright 
 *       notice, this list of conditions and the following disclaimer.
 * 
 *    2. Redistributions in binary form must reproduce the above copyright 
 *       notice, this list of conditions and the following disclaimer in 
 *       the documentation and/or other materials provided with the 
 *       distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY B. SCHAUERTE ''AS IS'' AND ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL B. SCHAUERTE OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  
 * The views and conclusions contained in the software and documentation
 * are those of the authors and should not be interpreted as representing 
 * official policies, either expressed or implied, of B. Schauerte.
 */

/**
 * If you use any of this work in scientific research or as part of a larger
 * software system, you are kindly requested to cite the use in any related 
 * publications or technical documentation. The work is based upon:
 *
 * [1] B. Schauerte, and R. Stiefelhagen, "Predicting Human Gaze using 
 *     Quaternion DCT Image Signature Saliency and Face Detection," in IEEE 
 *     Workshop on the Applications of Computer Vision (WACV), 2012.
 * [2] B. Schauerte, and R. Stiefelhagen, "Quaternion-based Spectral 
 *     Saliency Detection for Eye Fixation Prediction," in European 
 *     Conference on Computer Vision (ECCV), 2012
 */
#include "dct_type2.hpp"

#include <cstddef>				 // for size_t

#ifndef DK
#define DK(name, value) const E name = K(value)
#endif

#ifndef WS
#define WS(stride, i)  (stride * i)
#endif

#ifndef MAKE_VOLATILE_STRIDE
#define MAKE_VOLATILE_STRIDE(x) x
#endif

template <typename R, typename stride, typename INT>
void
dct_type3_64(const R * I, R * O, stride is, stride os, INT v, INT ivs, INT ovs)
{
	typedef R E;
	typedef R K;
	DK(KP1_999397637, +1.999397637392408440231531299332344393700122163);
	DK(KP049082457, +0.049082457045824576063469058918565850130932238);
	DK(KP1_379081089, +1.379081089474133849233461259914969405691073689);
	DK(KP1_448494165, +1.448494165902933841882138486581106334966186010);
	DK(KP1_997590912, +1.997590912410344785429543209518201388886407229);
	DK(KP098135348, +0.098135348654836028509909953885365316629490726);
	DK(KP1_984959069, +1.984959069197419996313534503322235640021641309);
	DK(KP244821350, +0.244821350398432396997408948301891575150447218);
	DK(KP1_230463181, +1.230463181161253690969827126827968555318860016);
	DK(KP1_576692855, +1.576692855253212524018329410719378565312986274);
	DK(KP1_940062506, +1.940062506389087985207968414572200502913731924);
	DK(KP485960359, +0.485960359806527779896548324154942236641981567);
	DK(KP1_927552131, +1.927552131590879733372928711015670307326167698);
	DK(KP533425514, +0.533425514949796772650573030232872788084233977);
	DK(KP985796384, +0.985796384459568073746053377517618536479374613);
	DK(KP1_740173982, +1.740173982217422837304584808967697687821655579);
	DK(KP1_715457220, +1.715457220000544139804539968569540274084981599);
	DK(KP1_028205488, +1.028205488386443453187387677937631545216098241);
	DK(KP1_763842528, +1.763842528696710059425513727320776699016885241);
	DK(KP942793473, +0.942793473651995297112775251810508755314920638);
	DK(KP1_865985597, +1.865985597669477775423320511086604996590031041);
	DK(KP719790073, +0.719790073069976297550209144653512840404634842);
	DK(KP810482628, +0.810482628009979741816962611010104933023895508);
	DK(KP1_828419511, +1.828419511407061309270029658787154802089382231);
	DK(KP1_481902250, +1.481902250709918182351233794990325459457910619);
	DK(KP1_343117909, +1.343117909694036801250753700854843606457501264);
	DK(KP1_990369453, +1.990369453344393772489673906218959843150949737);
	DK(KP196034280, +0.196034280659121203988391127777283691722273346);
	DK(KP1_970555284, +1.970555284777882489548036866357095574320258312);
	DK(KP341923777, +0.341923777520602452727284714416527063932658118);
	DK(KP1_151616382, +1.151616382835690601491944907631461683552016911);
	DK(KP1_635169626, +1.635169626303167393009841768261267618942085035);
	DK(KP1_883088130, +1.883088130366041556825018805199004714371179592);
	DK(KP673779706, +0.673779706784440101378506425238295140955533559);
	DK(KP1_994580913, +1.994580913357380432271194280365135642343377358);
	DK(KP147129127, +0.147129127199334847058931243150468643626598531);
	DK(KP1_306345685, +1.306345685907553528168406027312610830153720047);
	DK(KP1_514417693, +1.514417693012969095150928107211568946080867431);
	DK(KP1_978353019, +1.978353019929561946903347476032486127967379067);
	DK(KP293460948, +0.293460948910723503317700259293435639412430633);
	DK(KP1_899056361, +1.899056361186073334391872148378690056504448308);
	DK(KP627363480, +0.627363480797782953312957691988200619986755019);
	DK(KP899222659, +0.899222659309213200092589158848454151766374097);
	DK(KP1_786448602, +1.786448602391030640684832894986795956001251178);
	DK(KP1_606415062, +1.606415062961289819613353025926283847759138854);
	DK(KP1_191398608, +1.191398608984866686934073057659939779023852677);
	DK(KP1_913880671, +1.913880671464417729871595773960539938965698411);
	DK(KP580569354, +0.580569354508924735272384751634790549382952557);
	DK(KP1_951404260, +1.951404260077057088920791532839055943288024532);
	DK(KP438202480, +0.438202480313739594455475094994715597696721593);
	DK(KP1_069995239, +1.069995239774194421326153809274035831120531384);
	DK(KP1_689707130, +1.689707130499414146519142410209914195439571963);
	DK(KP1_807978586, +1.807978586246886663172400594461074097420264050);
	DK(KP855110186, +0.855110186860564188641933713777597068609157259);
	DK(KP1_546020906, +1.546020906725473921621813219516939601942082586);
	DK(KP1_268786568, +1.268786568327290996430343226450986741351374190);
	DK(KP471396736, +0.471396736825997648556387625905254377657460319);
	DK(KP881921264, +0.881921264348355029712756863660388349508442621);
	DK(KP098017140, +0.098017140329560601994195563888641845861136673);
	DK(KP995184726, +0.995184726672196886244836953109479921575474869);
	DK(KP290284677, +0.290284677254462367636192375817395274691476278);
	DK(KP956940335, +0.956940335732208864935797886980269969482849206);
	DK(KP634393284, +0.634393284163645498215171613225493370675687095);
	DK(KP773010453, +0.773010453362736960810906609758469800971041293);
	DK(KP555570233, +0.555570233019602224742830813948532874374937191);
	DK(KP831469612, +0.831469612302545237078788377617905756738560812);
	DK(KP980785280, +0.980785280403230449126182236134239036973933731);
	DK(KP195090322, +0.195090322016128267848284868477022240927691618);
	DK(KP382683432, +0.382683432365089771728459984030398866761344562);
	DK(KP923879532, +0.923879532511286756128183189396788286822416626);
	DK(KP1_662939224, +1.662939224605090474157576755235811513477121624);
	DK(KP1_111140466, +1.111140466039204449485661627897065748749874382);
	DK(KP1_961570560, +1.961570560806460898252364472268478073947867462);
	DK(KP390180644, +0.390180644032256535696569736954044481855383236);
	DK(KP707106781, +0.707106781186547524400844362104849039284835938);
	DK(KP765366864, +0.765366864730179543456919968060797733522689125);
	DK(KP1_847759065, +1.847759065022573512256366378793576573644833252);
	DK(KP1_414213562, +1.414213562373095048801688724209698078569671875);
	INT i;
	for (i = v; i > 0; i = i - 1, I = I + ivs, O = O + ovs, MAKE_VOLATILE_STRIDE(is), MAKE_VOLATILE_STRIDE(os))
	{
		E T8;
		E T214;
		E T284;
		E T390;
		E T17;
		E T215;
		E T287;
		E T391;
		E T31;
		E T217;
		E T294;
		E T394;
		E T38;
		E T218;
		E T291;
		E T393;
		E T99;
		E T229;
		E T352;
		E T420;
		E T176;
		E T244;
		E T319;
		E T405;
		E T53;
		E T221;
		E T313;
		E T401;
		E T83;
		E T225;
		E T299;
		E T397;
		E T72;
		E T224;
		E T310;
		E T398;
		E T76;
		E T222;
		E T306;
		E T400;
		E T118;
		E T243;
		E T349;
		E T406;
		E T169;
		E T230;
		E T326;
		E T419;
		E T161;
		E T165;
		E T413;
		E T417;
		E T237;
		E T241;
		E T341;
		E T345;
		E T140;
		E T164;
		E T410;
		E T416;
		E T234;
		E T240;
		E T334;
		E T344;
		{
			E T1;
			E T3;
			E T7;
			E T283;
			E T2;
			E T5;
			E T6;
			E T4;
			E T282;
			T1 = I[0];
			T2 = I[WS(is, 32)];
			T3 = KP1_414213562 * T2;
			T5 = I[WS(is, 16)];
			T6 = I[WS(is, 48)];
			T7 = FMA(KP1_847759065, T5, KP765366864 * T6);
			T283 = FNMS(KP1_847759065, T6, KP765366864 * T5);
			T4 = T1 + T3;
			T8 = T4 - T7;
			T214 = T4 + T7;
			T282 = T1 - T3;
			T284 = T282 - T283;
			T390 = T282 + T283;
		}
		{
			E T9;
			E T15;
			E T12;
			E T14;
			E T10;
			E T11;
			T9 = I[WS(is, 8)];
			T15 = I[WS(is, 56)];
			T10 = I[WS(is, 40)];
			T11 = I[WS(is, 24)];
			T12 = KP707106781 * (T10 + T11);
			T14 = KP707106781 * (T10 - T11);
			{
				E T13;
				E T16;
				E T285;
				E T286;
				T13 = T9 + T12;
				T16 = T14 - T15;
				T17 = FMA(KP390180644, T13, KP1_961570560 * T16);
				T215 = FNMS(KP390180644, T16, KP1_961570560 * T13);
				T285 = T9 - T12;
				T286 = T14 + T15;
				T287 = FMA(KP1_111140466, T285, KP1_662939224 * T286);
				T391 = FNMS(KP1_111140466, T286, KP1_662939224 * T285);
			}
		}
		{
			E T19;
			E T36;
			E T22;
			E T35;
			E T26;
			E T32;
			E T29;
			E T33;
			E T20;
			E T21;
			T19 = I[WS(is, 4)];
			T36 = I[WS(is, 60)];
			T20 = I[WS(is, 36)];
			T21 = I[WS(is, 28)];
			T22 = KP707106781 * (T20 + T21);
			T35 = KP707106781 * (T20 - T21);
			{
				E T24;
				E T25;
				E T27;
				E T28;
				T24 = I[WS(is, 20)];
				T25 = I[WS(is, 44)];
				T26 = FMA(KP923879532, T24, KP382683432 * T25);
				T32 = FNMS(KP923879532, T25, KP382683432 * T24);
				T27 = I[WS(is, 12)];
				T28 = I[WS(is, 52)];
				T29 = FMA(KP923879532, T27, KP382683432 * T28);
				T33 = FNMS(KP382683432, T27, KP923879532 * T28);
			}
			{
				E T23;
				E T30;
				E T292;
				E T293;
				T23 = T19 + T22;
				T30 = T26 + T29;
				T31 = T23 - T30;
				T217 = T23 + T30;
				T292 = T26 - T29;
				T293 = T35 + T36;
				T294 = T292 - T293;
				T394 = T292 + T293;
			}
			{
				E T34;
				E T37;
				E T289;
				E T290;
				T34 = T32 + T33;
				T37 = T35 - T36;
				T38 = T34 - T37;
				T218 = T34 + T37;
				T289 = T19 - T22;
				T290 = T32 - T33;
				T291 = T289 - T290;
				T393 = T289 + T290;
			}
		}
		{
			E T87;
			E T174;
			E T90;
			E T173;
			E T94;
			E T170;
			E T97;
			E T171;
			E T88;
			E T89;
			T87 = I[WS(is, 1)];
			T174 = I[WS(is, 63)];
			T88 = I[WS(is, 33)];
			T89 = I[WS(is, 31)];
			T90 = KP707106781 * (T88 + T89);
			T173 = KP707106781 * (T88 - T89);
			{
				E T92;
				E T93;
				E T95;
				E T96;
				T92 = I[WS(is, 17)];
				T93 = I[WS(is, 47)];
				T94 = FMA(KP923879532, T92, KP382683432 * T93);
				T170 = FNMS(KP923879532, T93, KP382683432 * T92);
				T95 = I[WS(is, 15)];
				T96 = I[WS(is, 49)];
				T97 = FMA(KP923879532, T95, KP382683432 * T96);
				T171 = FNMS(KP382683432, T95, KP923879532 * T96);
			}
			{
				E T91;
				E T98;
				E T350;
				E T351;
				T91 = T87 + T90;
				T98 = T94 + T97;
				T99 = T91 - T98;
				T229 = T91 + T98;
				T350 = T94 - T97;
				T351 = T173 + T174;
				T352 = T350 - T351;
				T420 = T350 + T351;
			}
			{
				E T172;
				E T175;
				E T317;
				E T318;
				T172 = T170 + T171;
				T175 = T173 - T174;
				T176 = T172 - T175;
				T244 = T172 + T175;
				T317 = T87 - T90;
				T318 = T170 - T171;
				T319 = T317 - T318;
				T405 = T317 + T318;
			}
		}
		{
			E T41;
			E T81;
			E T44;
			E T80;
			E T48;
			E T77;
			E T51;
			E T78;
			E T42;
			E T43;
			T41 = I[WS(is, 2)];
			T81 = I[WS(is, 62)];
			T42 = I[WS(is, 34)];
			T43 = I[WS(is, 30)];
			T44 = KP707106781 * (T42 + T43);
			T80 = KP707106781 * (T42 - T43);
			{
				E T46;
				E T47;
				E T49;
				E T50;
				T46 = I[WS(is, 18)];
				T47 = I[WS(is, 46)];
				T48 = FMA(KP923879532, T46, KP382683432 * T47);
				T77 = FNMS(KP923879532, T47, KP382683432 * T46);
				T49 = I[WS(is, 14)];
				T50 = I[WS(is, 50)];
				T51 = FMA(KP923879532, T49, KP382683432 * T50);
				T78 = FNMS(KP382683432, T49, KP923879532 * T50);
			}
			{
				E T45;
				E T52;
				E T311;
				E T312;
				T45 = T41 + T44;
				T52 = T48 + T51;
				T53 = T45 - T52;
				T221 = T45 + T52;
				T311 = T48 - T51;
				T312 = T80 + T81;
				T313 = T311 - T312;
				T401 = T311 + T312;
			}
			{
				E T79;
				E T82;
				E T297;
				E T298;
				T79 = T77 + T78;
				T82 = T80 - T81;
				T83 = T79 - T82;
				T225 = T79 + T82;
				T297 = T41 - T44;
				T298 = T77 - T78;
				T299 = T297 - T298;
				T397 = T297 + T298;
			}
		}
		{
			E T58;
			E T301;
			E T61;
			E T300;
			E T67;
			E T304;
			E T70;
			E T303;
			{
				E T54;
				E T60;
				E T57;
				E T59;
				E T55;
				E T56;
				T54 = I[WS(is, 10)];
				T60 = I[WS(is, 54)];
				T55 = I[WS(is, 42)];
				T56 = I[WS(is, 22)];
				T57 = KP707106781 * (T55 + T56);
				T59 = KP707106781 * (T55 - T56);
				T58 = T54 + T57;
				T301 = T59 + T60;
				T61 = T59 - T60;
				T300 = T54 - T57;
			}
			{
				E T66;
				E T68;
				E T65;
				E T69;
				E T63;
				E T64;
				T66 = I[WS(is, 58)];
				T68 = I[WS(is, 6)];
				T63 = I[WS(is, 26)];
				T64 = I[WS(is, 38)];
				T65 = KP707106781 * (T63 - T64);
				T69 = KP707106781 * (T63 + T64);
				T67 = T65 + T66;
				T304 = T68 - T69;
				T70 = T68 + T69;
				T303 = T65 - T66;
			}
			{
				E T62;
				E T71;
				E T308;
				E T309;
				T62 = FMA(KP195090322, T58, KP980785280 * T61);
				T71 = FNMS(KP195090322, T70, KP980785280 * T67);
				T72 = T62 - T71;
				T224 = T62 + T71;
				T308 = FNMS(KP555570233, T301, KP831469612 * T300);
				T309 = FMA(KP831469612, T304, KP555570233 * T303);
				T310 = T308 - T309;
				T398 = T308 + T309;
			}
			{
				E T74;
				E T75;
				E T302;
				E T305;
				T74 = FNMS(KP195090322, T61, KP980785280 * T58);
				T75 = FMA(KP980785280, T70, KP195090322 * T67);
				T76 = T74 - T75;
				T222 = T74 + T75;
				T302 = FMA(KP555570233, T300, KP831469612 * T301);
				T305 = FNMS(KP555570233, T304, KP831469612 * T303);
				T306 = T302 - T305;
				T400 = T302 + T305;
			}
		}
		{
			E T104;
			E T321;
			E T107;
			E T320;
			E T113;
			E T324;
			E T116;
			E T323;
			{
				E T100;
				E T106;
				E T103;
				E T105;
				E T101;
				E T102;
				T100 = I[WS(is, 9)];
				T106 = I[WS(is, 55)];
				T101 = I[WS(is, 41)];
				T102 = I[WS(is, 23)];
				T103 = KP707106781 * (T101 + T102);
				T105 = KP707106781 * (T101 - T102);
				T104 = T100 + T103;
				T321 = T105 + T106;
				T107 = T105 - T106;
				T320 = T100 - T103;
			}
			{
				E T112;
				E T114;
				E T111;
				E T115;
				E T109;
				E T110;
				T112 = I[WS(is, 57)];
				T114 = I[WS(is, 7)];
				T109 = I[WS(is, 25)];
				T110 = I[WS(is, 39)];
				T111 = KP707106781 * (T109 - T110);
				T115 = KP707106781 * (T109 + T110);
				T113 = T111 + T112;
				T324 = T114 - T115;
				T116 = T114 + T115;
				T323 = T111 - T112;
			}
			{
				E T108;
				E T117;
				E T347;
				E T348;
				T108 = FMA(KP195090322, T104, KP980785280 * T107);
				T117 = FNMS(KP195090322, T116, KP980785280 * T113);
				T118 = T108 - T117;
				T243 = T108 + T117;
				T347 = FNMS(KP555570233, T321, KP831469612 * T320);
				T348 = FMA(KP831469612, T324, KP555570233 * T323);
				T349 = T347 - T348;
				T406 = T347 + T348;
			}
			{
				E T167;
				E T168;
				E T322;
				E T325;
				T167 = FNMS(KP195090322, T107, KP980785280 * T104);
				T168 = FMA(KP980785280, T116, KP195090322 * T113);
				T169 = T167 - T168;
				T230 = T167 + T168;
				T322 = FMA(KP555570233, T320, KP831469612 * T321);
				T325 = FNMS(KP555570233, T324, KP831469612 * T323);
				T326 = T322 - T325;
				T419 = T322 + T325;
			}
		}
		{
			E T152;
			E T338;
			E T156;
			E T336;
			E T147;
			E T339;
			E T159;
			E T335;
			{
				E T151;
				E T154;
				E T150;
				E T155;
				E T148;
				E T149;
				T151 = I[WS(is, 61)];
				T154 = I[WS(is, 3)];
				T148 = I[WS(is, 29)];
				T149 = I[WS(is, 35)];
				T150 = KP707106781 * (T148 - T149);
				T155 = KP707106781 * (T148 + T149);
				T152 = T150 + T151;
				T338 = T154 - T155;
				T156 = T154 + T155;
				T336 = T150 - T151;
			}
			{
				E T143;
				E T157;
				E T146;
				E T158;
				{
					E T141;
					E T142;
					E T144;
					E T145;
					T141 = I[WS(is, 13)];
					T142 = I[WS(is, 51)];
					T143 = FNMS(KP923879532, T142, KP382683432 * T141);
					T157 = FMA(KP923879532, T141, KP382683432 * T142);
					T144 = I[WS(is, 45)];
					T145 = I[WS(is, 19)];
					T146 = FNMS(KP382683432, T145, KP923879532 * T144);
					T158 = FMA(KP923879532, T145, KP382683432 * T144);
				}
				T147 = T143 + T146;
				T339 = T143 - T146;
				T159 = T157 + T158;
				T335 = T157 - T158;
			}
			{
				E T153;
				E T160;
				E T411;
				E T412;
				T153 = T147 - T152;
				T160 = T156 - T159;
				T161 = FNMS(KP634393284, T160, KP773010453 * T153);
				T165 = FMA(KP773010453, T160, KP634393284 * T153);
				T411 = T335 + T336;
				T412 = T338 + T339;
				T413 = FNMS(KP290284677, T412, KP956940335 * T411);
				T417 = FMA(KP956940335, T412, KP290284677 * T411);
			}
			{
				E T235;
				E T236;
				E T337;
				E T340;
				T235 = T147 + T152;
				T236 = T156 + T159;
				T237 = FNMS(KP098017140, T236, KP995184726 * T235);
				T241 = FMA(KP995184726, T236, KP098017140 * T235);
				T337 = T335 - T336;
				T340 = T338 - T339;
				T341 = FNMS(KP471396736, T340, KP881921264 * T337);
				T345 = FMA(KP881921264, T340, KP471396736 * T337);
			}
		}
		{
			E T124;
			E T332;
			E T138;
			E T328;
			E T131;
			E T331;
			E T135;
			E T329;
			{
				E T120;
				E T137;
				E T123;
				E T136;
				E T121;
				E T122;
				T120 = I[WS(is, 5)];
				T137 = I[WS(is, 59)];
				T121 = I[WS(is, 37)];
				T122 = I[WS(is, 27)];
				T123 = KP707106781 * (T121 + T122);
				T136 = KP707106781 * (T121 - T122);
				T124 = T120 + T123;
				T332 = T136 + T137;
				T138 = T136 - T137;
				T328 = T120 - T123;
			}
			{
				E T127;
				E T133;
				E T130;
				E T134;
				{
					E T125;
					E T126;
					E T128;
					E T129;
					T125 = I[WS(is, 21)];
					T126 = I[WS(is, 43)];
					T127 = FMA(KP923879532, T125, KP382683432 * T126);
					T133 = FNMS(KP923879532, T126, KP382683432 * T125);
					T128 = I[WS(is, 11)];
					T129 = I[WS(is, 53)];
					T130 = FMA(KP923879532, T128, KP382683432 * T129);
					T134 = FNMS(KP382683432, T128, KP923879532 * T129);
				}
				T131 = T127 + T130;
				T331 = T127 - T130;
				T135 = T133 + T134;
				T329 = T133 - T134;
			}
			{
				E T132;
				E T139;
				E T408;
				E T409;
				T132 = T124 - T131;
				T139 = T135 - T138;
				T140 = FMA(KP634393284, T132, KP773010453 * T139);
				T164 = FNMS(KP634393284, T139, KP773010453 * T132);
				T408 = T328 + T329;
				T409 = T331 + T332;
				T410 = FMA(KP290284677, T408, KP956940335 * T409);
				T416 = FNMS(KP290284677, T409, KP956940335 * T408);
			}
			{
				E T232;
				E T233;
				E T330;
				E T333;
				T232 = T124 + T131;
				T233 = T135 + T138;
				T234 = FMA(KP098017140, T232, KP995184726 * T233);
				T240 = FNMS(KP098017140, T233, KP995184726 * T232);
				T330 = T328 - T329;
				T333 = T331 - T332;
				T334 = FMA(KP471396736, T330, KP881921264 * T333);
				T344 = FNMS(KP471396736, T333, KP881921264 * T330);
			}
		}
		{
			E T40;
			E T182;
			E T178;
			E T186;
			E T85;
			E T183;
			E T163;
			E T185;
			{
				E T18;
				E T39;
				E T166;
				E T177;
				T18 = T8 - T17;
				T39 = FMA(KP1_268786568, T31, KP1_546020906 * T38);
				T40 = T18 - T39;
				T182 = T18 + T39;
				T166 = T164 - T165;
				T177 = T169 - T176;
				T178 = T166 - T177;
				T186 = T166 + T177;
			}
			{
				E T73;
				E T84;
				E T119;
				E T162;
				T73 = T53 - T72;
				T84 = T76 - T83;
				T85 = FMA(KP855110186, T73, KP1_807978586 * T84);
				T183 = FNMS(KP855110186, T84, KP1_807978586 * T73);
				T119 = T99 - T118;
				T162 = T140 - T161;
				T163 = T119 - T162;
				T185 = T119 + T162;
			}
			{
				E T86;
				E T179;
				E T188;
				E T189;
				T86 = T40 + T85;
				T179 = FNMS(KP1_069995239, T178, KP1_689707130 * T163);
				O[WS(os, 52)] = T86 - T179;
				O[WS(os, 11)] = T86 + T179;
				T188 = T182 - T183;
				T189 = FMA(KP438202480, T185, KP1_951404260 * T186);
				O[WS(os, 36)] = T188 - T189;
				O[WS(os, 27)] = T188 + T189;
			}
			{
				E T180;
				E T181;
				E T184;
				E T187;
				T180 = T40 - T85;
				T181 = FMA(KP1_069995239, T163, KP1_689707130 * T178);
				O[WS(os, 43)] = T180 - T181;
				O[WS(os, 20)] = T180 + T181;
				T184 = T182 + T183;
				T187 = FNMS(KP438202480, T186, KP1_951404260 * T185);
				O[WS(os, 59)] = T184 - T187;
				O[WS(os, 4)] = T184 + T187;
			}
		}
		{
			E T396;
			E T426;
			E T422;
			E T430;
			E T403;
			E T427;
			E T415;
			E T429;
			{
				E T392;
				E T395;
				E T418;
				E T421;
				T392 = T390 - T391;
				T395 = FMA(KP580569354, T393, KP1_913880671 * T394);
				T396 = T392 - T395;
				T426 = T392 + T395;
				T418 = T416 - T417;
				T421 = T419 - T420;
				T422 = T418 - T421;
				T430 = T418 + T421;
			}
			{
				E T399;
				E T402;
				E T407;
				E T414;
				T399 = T397 - T398;
				T402 = T400 - T401;
				T403 = FMA(KP1_191398608, T399, KP1_606415062 * T402);
				T427 = FNMS(KP1_191398608, T402, KP1_606415062 * T399);
				T407 = T405 - T406;
				T414 = T410 - T413;
				T415 = T407 - T414;
				T429 = T407 + T414;
			}
			{
				E T404;
				E T423;
				E T432;
				E T433;
				T404 = T396 + T403;
				T423 = FNMS(KP899222659, T422, KP1_786448602 * T415);
				O[WS(os, 54)] = T404 - T423;
				O[WS(os, 9)] = T404 + T423;
				T432 = T426 - T427;
				T433 = FMA(KP627363480, T429, KP1_899056361 * T430);
				O[WS(os, 38)] = T432 - T433;
				O[WS(os, 25)] = T432 + T433;
			}
			{
				E T424;
				E T425;
				E T428;
				E T431;
				T424 = T396 - T403;
				T425 = FMA(KP899222659, T415, KP1_786448602 * T422);
				O[WS(os, 41)] = T424 - T425;
				O[WS(os, 22)] = T424 + T425;
				T428 = T426 + T427;
				T431 = FNMS(KP627363480, T430, KP1_899056361 * T429);
				O[WS(os, 57)] = T428 - T431;
				O[WS(os, 6)] = T428 + T431;
			}
		}
		{
			E T436;
			E T450;
			E T446;
			E T454;
			E T439;
			E T451;
			E T443;
			E T453;
			{
				E T434;
				E T435;
				E T444;
				E T445;
				T434 = T390 + T391;
				T435 = FNMS(KP580569354, T394, KP1_913880671 * T393);
				T436 = T434 - T435;
				T450 = T434 + T435;
				T444 = T410 + T413;
				T445 = T419 + T420;
				T446 = T444 - T445;
				T454 = T444 + T445;
			}
			{
				E T437;
				E T438;
				E T441;
				E T442;
				T437 = T397 + T398;
				T438 = T400 + T401;
				T439 = FMA(KP293460948, T437, KP1_978353019 * T438);
				T451 = FNMS(KP293460948, T438, KP1_978353019 * T437);
				T441 = T405 + T406;
				T442 = T416 + T417;
				T443 = T441 - T442;
				T453 = T441 + T442;
			}
			{
				E T440;
				E T447;
				E T456;
				E T457;
				T440 = T436 + T439;
				T447 = FNMS(KP1_306345685, T446, KP1_514417693 * T443);
				O[WS(os, 49)] = T440 - T447;
				O[WS(os, 14)] = T440 + T447;
				T456 = T450 - T451;
				T457 = FMA(KP147129127, T453, KP1_994580913 * T454);
				O[WS(os, 33)] = T456 - T457;
				O[WS(os, 30)] = T456 + T457;
			}
			{
				E T448;
				E T449;
				E T452;
				E T455;
				T448 = T436 - T439;
				T449 = FMA(KP1_306345685, T443, KP1_514417693 * T446);
				O[WS(os, 46)] = T448 - T449;
				O[WS(os, 17)] = T448 + T449;
				T452 = T450 + T451;
				T455 = FNMS(KP147129127, T454, KP1_994580913 * T453);
				O[WS(os, 62)] = T452 - T455;
				O[WS(os, 1)] = T452 + T455;
			}
		}
		{
			E T192;
			E T206;
			E T202;
			E T210;
			E T195;
			E T207;
			E T199;
			E T209;
			{
				E T190;
				E T191;
				E T200;
				E T201;
				T190 = T8 + T17;
				T191 = FNMS(KP1_268786568, T38, KP1_546020906 * T31);
				T192 = T190 - T191;
				T206 = T190 + T191;
				T200 = T140 + T161;
				T201 = T169 + T176;
				T202 = T200 - T201;
				T210 = T200 + T201;
			}
			{
				E T193;
				E T194;
				E T197;
				E T198;
				T193 = T53 + T72;
				T194 = T76 + T83;
				T195 = FMA(KP673779706, T193, KP1_883088130 * T194);
				T207 = FNMS(KP673779706, T194, KP1_883088130 * T193);
				T197 = T99 + T118;
				T198 = T164 + T165;
				T199 = T197 - T198;
				T209 = T197 + T198;
			}
			{
				E T196;
				E T203;
				E T212;
				E T213;
				T196 = T192 + T195;
				T203 = FNMS(KP1_151616382, T202, KP1_635169626 * T199);
				O[WS(os, 51)] = T196 - T203;
				O[WS(os, 12)] = T196 + T203;
				T212 = T206 - T207;
				T213 = FMA(KP341923777, T209, KP1_970555284 * T210);
				O[WS(os, 35)] = T212 - T213;
				O[WS(os, 28)] = T212 + T213;
			}
			{
				E T204;
				E T205;
				E T208;
				E T211;
				T204 = T192 - T195;
				T205 = FMA(KP1_151616382, T199, KP1_635169626 * T202);
				O[WS(os, 44)] = T204 - T205;
				O[WS(os, 19)] = T204 + T205;
				T208 = T206 + T207;
				T211 = FNMS(KP341923777, T210, KP1_970555284 * T209);
				O[WS(os, 60)] = T208 - T211;
				O[WS(os, 3)] = T208 + T211;
			}
		}
		{
			E T220;
			E T250;
			E T246;
			E T254;
			E T227;
			E T251;
			E T239;
			E T253;
			{
				E T216;
				E T219;
				E T242;
				E T245;
				T216 = T214 - T215;
				T219 = FMA(KP196034280, T217, KP1_990369453 * T218);
				T220 = T216 - T219;
				T250 = T216 + T219;
				T242 = T240 - T241;
				T245 = T243 - T244;
				T246 = T242 - T245;
				T254 = T242 + T245;
			}
			{
				E T223;
				E T226;
				E T231;
				E T238;
				T223 = T221 - T222;
				T226 = T224 - T225;
				T227 = FMA(KP1_343117909, T223, KP1_481902250 * T226);
				T251 = FNMS(KP1_343117909, T226, KP1_481902250 * T223);
				T231 = T229 - T230;
				T238 = T234 - T237;
				T239 = T231 - T238;
				T253 = T231 + T238;
			}
			{
				E T228;
				E T247;
				E T256;
				E T257;
				T228 = T220 + T227;
				T247 = FNMS(KP810482628, T246, KP1_828419511 * T239);
				O[WS(os, 55)] = T228 - T247;
				O[WS(os, 8)] = T228 + T247;
				T256 = T250 - T251;
				T257 = FMA(KP719790073, T253, KP1_865985597 * T254);
				O[WS(os, 39)] = T256 - T257;
				O[WS(os, 24)] = T256 + T257;
			}
			{
				E T248;
				E T249;
				E T252;
				E T255;
				T248 = T220 - T227;
				T249 = FMA(KP810482628, T239, KP1_828419511 * T246);
				O[WS(os, 40)] = T248 - T249;
				O[WS(os, 23)] = T248 + T249;
				T252 = T250 + T251;
				T255 = FNMS(KP719790073, T254, KP1_865985597 * T253);
				O[WS(os, 56)] = T252 - T255;
				O[WS(os, 7)] = T252 + T255;
			}
		}
		{
			E T296;
			E T358;
			E T354;
			E T362;
			E T315;
			E T359;
			E T343;
			E T361;
			{
				E T288;
				E T295;
				E T346;
				E T353;
				T288 = T284 - T287;
				T295 = FMA(KP942793473, T291, KP1_763842528 * T294);
				T296 = T288 - T295;
				T358 = T288 + T295;
				T346 = T344 - T345;
				T353 = T349 - T352;
				T354 = T346 - T353;
				T362 = T346 + T353;
			}
			{
				E T307;
				E T314;
				E T327;
				E T342;
				T307 = T299 - T306;
				T314 = T310 - T313;
				T315 = FMA(KP1_028205488, T307, KP1_715457220 * T314);
				T359 = FNMS(KP1_028205488, T314, KP1_715457220 * T307);
				T327 = T319 - T326;
				T342 = T334 - T341;
				T343 = T327 - T342;
				T361 = T327 + T342;
			}
			{
				E T316;
				E T355;
				E T364;
				E T365;
				T316 = T296 + T315;
				T355 = FNMS(KP985796384, T354, KP1_740173982 * T343);
				O[WS(os, 53)] = T316 - T355;
				O[WS(os, 10)] = T316 + T355;
				T364 = T358 - T359;
				T365 = FMA(KP533425514, T361, KP1_927552131 * T362);
				O[WS(os, 37)] = T364 - T365;
				O[WS(os, 26)] = T364 + T365;
			}
			{
				E T356;
				E T357;
				E T360;
				E T363;
				T356 = T296 - T315;
				T357 = FMA(KP985796384, T343, KP1_740173982 * T354);
				O[WS(os, 42)] = T356 - T357;
				O[WS(os, 21)] = T356 + T357;
				T360 = T358 + T359;
				T363 = FNMS(KP533425514, T362, KP1_927552131 * T361);
				O[WS(os, 58)] = T360 - T363;
				O[WS(os, 5)] = T360 + T363;
			}
		}
		{
			E T368;
			E T382;
			E T378;
			E T386;
			E T371;
			E T383;
			E T375;
			E T385;
			{
				E T366;
				E T367;
				E T376;
				E T377;
				T366 = T284 + T287;
				T367 = FNMS(KP942793473, T294, KP1_763842528 * T291);
				T368 = T366 - T367;
				T382 = T366 + T367;
				T376 = T334 + T341;
				T377 = T349 + T352;
				T378 = T376 - T377;
				T386 = T376 + T377;
			}
			{
				E T369;
				E T370;
				E T373;
				E T374;
				T369 = T299 + T306;
				T370 = T310 + T313;
				T371 = FMA(KP485960359, T369, KP1_940062506 * T370);
				T383 = FNMS(KP485960359, T370, KP1_940062506 * T369);
				T373 = T319 + T326;
				T374 = T344 + T345;
				T375 = T373 - T374;
				T385 = T373 + T374;
			}
			{
				E T372;
				E T379;
				E T388;
				E T389;
				T372 = T368 + T371;
				T379 = FNMS(KP1_230463181, T378, KP1_576692855 * T375);
				O[WS(os, 50)] = T372 - T379;
				O[WS(os, 13)] = T372 + T379;
				T388 = T382 - T383;
				T389 = FMA(KP244821350, T385, KP1_984959069 * T386);
				O[WS(os, 34)] = T388 - T389;
				O[WS(os, 29)] = T388 + T389;
			}
			{
				E T380;
				E T381;
				E T384;
				E T387;
				T380 = T368 - T371;
				T381 = FMA(KP1_230463181, T375, KP1_576692855 * T378);
				O[WS(os, 45)] = T380 - T381;
				O[WS(os, 18)] = T380 + T381;
				T384 = T382 + T383;
				T387 = FNMS(KP244821350, T386, KP1_984959069 * T385);
				O[WS(os, 61)] = T384 - T387;
				O[WS(os, 2)] = T384 + T387;
			}
		}
		{
			E T260;
			E T274;
			E T270;
			E T278;
			E T263;
			E T275;
			E T267;
			E T277;
			{
				E T258;
				E T259;
				E T268;
				E T269;
				T258 = T214 + T215;
				T259 = FNMS(KP196034280, T218, KP1_990369453 * T217);
				T260 = T258 - T259;
				T274 = T258 + T259;
				T268 = T234 + T237;
				T269 = T243 + T244;
				T270 = T268 - T269;
				T278 = T268 + T269;
			}
			{
				E T261;
				E T262;
				E T265;
				E T266;
				T261 = T221 + T222;
				T262 = T224 + T225;
				T263 = FMA(KP098135348, T261, KP1_997590912 * T262);
				T275 = FNMS(KP098135348, T262, KP1_997590912 * T261);
				T265 = T229 + T230;
				T266 = T240 + T241;
				T267 = T265 - T266;
				T277 = T265 + T266;
			}
			{
				E T264;
				E T271;
				E T280;
				E T281;
				T264 = T260 + T263;
				T271 = FNMS(KP1_379081089, T270, KP1_448494165 * T267);
				O[WS(os, 48)] = T264 - T271;
				O[WS(os, 15)] = T264 + T271;
				T280 = T274 - T275;
				T281 = FMA(KP049082457, T277, KP1_999397637 * T278);
				O[WS(os, 32)] = T280 - T281;
				O[WS(os, 31)] = T280 + T281;
			}
			{
				E T272;
				E T273;
				E T276;
				E T279;
				T272 = T260 - T263;
				T273 = FMA(KP1_379081089, T267, KP1_448494165 * T270);
				O[WS(os, 47)] = T272 - T273;
				O[WS(os, 16)] = T272 + T273;
				T276 = T274 + T275;
				T279 = FNMS(KP049082457, T278, KP1_999397637 * T277);
				O[WS(os, 63)] = T276 - T279;
				O[0] = T276 + T279;
			}
		}
	}
}


template void dct_type3_64<>(const double*, double*, int, int, int, int, int);
template void dct_type3_64<>(const float*, float*, int, int, int, int, int);
template void dct_type3_64<>(const double*, double*, size_t, size_t, size_t, size_t, size_t);
template void dct_type3_64<>(const float*, float*, size_t, size_t, size_t, size_t, size_t);
