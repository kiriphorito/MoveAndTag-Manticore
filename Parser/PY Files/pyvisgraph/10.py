import pyvisgraph as vg
polys = [[vg.Point(-0.0246174585342621,0.16891705339831045),vg.Point(1.9301998869031127,-0.25379933613156713),vg.Point(2.1415580816680513,0.7236093365871203),vg.Point(1.1641494089493643,0.9349675313520591),vg.Point(1.5868657984792418,2.8897848767894345),vg.Point(0.6094571257605544,3.1011430715543726),vg.Point(0.39809893099561555,2.123734398835685),vg.Point(-0.5793097417230719,2.335092593600624),vg.Point(-0.15659335219319442,4.289909939037998),vg.Point(-1.1340020249118816,4.501268133802937),vg.Point(-1.7680766092066984,1.5690421156468752),vg.Point(0.18674073623067672,1.146325726116998)],[vg.Point(-0.05901157522490006,-0.09731020795587667),vg.Point(-1.492534431882603,-1.4919471569285974),vg.Point(-0.7952159573962381,-2.2087085852574506),vg.Point(-0.0784545290673882,-1.5113901107710888),vg.Point(1.3161824199053331,-2.9449129674287895),vg.Point(2.0329438482341837,-2.247594492942429),vg.Point(1.335625373747822,-1.5308330646135777),vg.Point(2.052386802076673,-0.8335145901272166),vg.Point(0.638306899261461,-0.8140716362847271)],[vg.Point(-1.8985138332922995,0.07593614482383877),vg.Point(-3.888733837631598,0.27348192262654597),vg.Point(-3.987506726532953,-0.7216280795431058),vg.Point(-2.992396724363303,-0.8204009684444589),vg.Point(-3.189942502166012,-2.8106209727837586),vg.Point(-2.194832499996357,-2.9093938616851127),vg.Point(-2.0960596110950043,-1.9142838595154614),vg.Point(-1.100949608925354,-2.013056748416814),vg.Point(-1.9972867221936528,-0.9191738573458109)],[vg.Point(3.558366313174796,-0.7469747877882957),vg.Point(4.200548350375849,-2.641071467236391),vg.Point(5.147596690099898,-2.3199804486358633),vg.Point(4.826505671499372,-1.3729321089118178),vg.Point(6.720602350947466,-0.7307500717107629),vg.Point(6.39951133234694,0.21629826801328667),vg.Point(5.452462992622889,-0.10479275058724113),vg.Point(5.1313719740223664,0.8422555891368066),vg.Point(4.5054146528988435,-0.4258837691877702)],[vg.Point(-0.1292924782198224,-2.838239291135715),vg.Point(-1.8695413496807514,-3.8239034658823434),vg.Point(-1.3767092623074424,-4.6940279016128095),vg.Point(-0.5065848265769743,-4.201195814239495),vg.Point(0.47907934816965103,-5.941444685700426),vg.Point(1.3492037839001179,-5.448612598327115),vg.Point(0.8563716965268057,-4.5784881625966465),vg.Point(1.72649613225727,-4.0856560752233335),vg.Point(0.36353960915349137,-3.70836372686618)],[vg.Point(0.25492505294213214,6.481748034941264),vg.Point(0.21323463991774919,8.481313465093988),vg.Point(-1.786330790234977,8.439623052069605)],[vg.Point(1.802666880196373,2.3290618407232597),vg.Point(2.727656395595662,0.5558178088987102),vg.Point(3.6142784115079385,1.0183125665983537),vg.Point(3.151783653808293,1.904934582510628),vg.Point(4.925027685632841,2.8299240979099256),vg.Point(4.462532927933195,3.7165461138222016),vg.Point(3.5759109120209187,3.2540513561225515),vg.Point(3.1134161543212775,4.140673372034827),vg.Point(2.6892888961086463,2.791556598422905)],[vg.Point(3.9747552446326915,6.354629051101332),vg.Point(2.451543302495492,5.0585482745559),vg.Point(3.0995836907682035,4.296942303487298),vg.Point(3.861189661836809,4.944982691760014),vg.Point(5.15727043838224,3.42177074962281),vg.Point(5.918876409450841,4.0698111378955275),vg.Point(5.2708360211781216,4.83141710896413),vg.Point(6.032441992246724,5.47945749723684),vg.Point(7.328522768792157,3.956245555099641),vg.Point(8.090128739860756,4.604285943372348),vg.Point(7.442088351588037,5.365891914440944),vg.Point(8.203694322656643,6.01393230271367),vg.Point(6.794047963315328,6.12749788550956),vg.Point(6.1460075750426135,6.889103856578155),vg.Point(4.622795632905406,5.593023080032731)],[vg.Point(-3.775070566373749,2.0380637954647742),vg.Point(-3.5175923907117252,4.021420795366142),vg.Point(-4.509270890662411,4.150159883197155),vg.Point(-4.638009978493424,3.158481383246468),vg.Point(-6.621366978394792,3.4159595589084892),vg.Point(-6.7501060662258014,2.4242810589578054),vg.Point(-5.758427566275118,2.295541971126795),vg.Point(-5.88716665410613,1.3038634711761112),vg.Point(-4.766749066324433,2.1668028832957846)],[vg.Point(-2.8928868598181494,1.3522478460536007),vg.Point(-4.84903978333255,1.7687400306454113),vg.Point(-5.265531967924365,-0.18741289286898621)],[vg.Point(-3.8010223726087475,-4.555270651862383),vg.Point(-4.295555878780673,-2.6173757601633265),vg.Point(-6.233450770479729,-3.1119092663352483)]]
g = vg.VisGraph()
g.build(polys)
shortest = g.shortest_path(vg.Point(2.2073865752517436,-1.889047496129887), vg.Point(7.418773235738816,-0.44990765706479596))
shortest += g.shortest_path(vg.Point(7.418773235738816,-0.44990765706479596), vg.Point(4.973369588410079,7.5255867963970475))
shortest += g.shortest_path(vg.Point(4.973369588410079,7.5255867963970475), vg.Point(-6.343762246568933,6.482770565702903))
shortest += g.shortest_path(vg.Point(-6.343762246568933,6.482770565702903), vg.Point(7.638112697692372,7.546541304735052))
shortest += g.shortest_path(vg.Point(7.638112697692372,7.546541304735052), vg.Point(-6.616019419392999,0.6818317478052967))
shortest += g.shortest_path(vg.Point(-6.616019419392999,0.6818317478052967), vg.Point(7.566930030403939,8.35536654882807))
shortest += g.shortest_path(vg.Point(7.566930030403939,8.35536654882807), vg.Point(1.962511182428952,3.827257744484897))
shortest += g.shortest_path(vg.Point(1.962511182428952,3.827257744484897), vg.Point(0.26424382924462364,4.408587560180833))
shortest += g.shortest_path(vg.Point(0.26424382924462364,4.408587560180833), vg.Point(-1.6363879702350923,4.2742016914298455))
shortest += g.shortest_path(vg.Point(-1.6363879702350923,4.2742016914298455), vg.Point(0.7514081303472473,3.0733529751116455))
shortest += g.shortest_path(vg.Point(0.7514081303472473,3.0733529751116455), vg.Point(0.5267146642115579,-0.4931108621138689))
shortest += g.shortest_path(vg.Point(0.5267146642115579,-0.4931108621138689), vg.Point(4.659784708236221,3.526482298351895))
shortest += g.shortest_path(vg.Point(4.659784708236221,3.526482298351895), vg.Point(7.758024410082515,-3.9166527369821686))
shortest += g.shortest_path(vg.Point(7.758024410082515,-3.9166527369821686), vg.Point(-3.768419738963783,5.061463126225703))
shortest += g.shortest_path(vg.Point(-3.768419738963783,5.061463126225703), vg.Point(-1.238311811529301,0.7476069382605006))
shortest += g.shortest_path(vg.Point(-1.238311811529301,0.7476069382605006), vg.Point(4.667157304771778,-2.534219407619858))
shortest += g.shortest_path(vg.Point(4.667157304771778,-2.534219407619858), vg.Point(-3.276919255050708,-2.3398760145109065))
shortest += g.shortest_path(vg.Point(-3.276919255050708,-2.3398760145109065), vg.Point(4.027309841872726,2.225932179379771))
shortest += g.shortest_path(vg.Point(4.027309841872726,2.225932179379771), vg.Point(-3.134941746562891,4.392880537243911))
shortest += g.shortest_path(vg.Point(-3.134941746562891,4.392880537243911), vg.Point(-1.7919488323765025,-0.9304890692153593))
shortest += g.shortest_path(vg.Point(-1.7919488323765025,-0.9304890692153593), vg.Point(-2.1055915905313096,7.17999624927331))
shortest += g.shortest_path(vg.Point(-2.1055915905313096,7.17999624927331), vg.Point(3.996028251723197,-2.2093291846560725))
shortest += g.shortest_path(vg.Point(3.996028251723197,-2.2093291846560725), vg.Point(-3.5164396704835954,6.373383887540029))
shortest += g.shortest_path(vg.Point(-3.5164396704835954,6.373383887540029), vg.Point(0.2854132295569878,5.243555165301994))
shortest += g.shortest_path(vg.Point(0.2854132295569878,5.243555165301994), vg.Point(-3.8098482367621416,0.7147790107512577))
shortest += g.shortest_path(vg.Point(-3.8098482367621416,0.7147790107512577), vg.Point(4.154214433161562,1.2618722266885047))
shortest += g.shortest_path(vg.Point(4.154214433161562,1.2618722266885047), vg.Point(1.7474003547927728,-1.2477990406749173))
shortest += g.shortest_path(vg.Point(1.7474003547927728,-1.2477990406749173), vg.Point(0.8212165138124101,7.303175605155082))
shortest += g.shortest_path(vg.Point(0.8212165138124101,7.303175605155082), vg.Point(4.302652030414358,4.117179861754971))
shortest += g.shortest_path(vg.Point(4.302652030414358,4.117179861754971), vg.Point(-5.4248143794161745,8.136563438916923))
shortest += g.shortest_path(vg.Point(-5.4248143794161745,8.136563438916923), vg.Point(-3.3165405488513895,6.68797610264568))
shortest += g.shortest_path(vg.Point(-3.3165405488513895,6.68797610264568), vg.Point(-3.030985979887064,-5.606359093806035))
shortest += g.shortest_path(vg.Point(-3.030985979887064,-5.606359093806035), vg.Point(1.7286467154371232,-3.3592566534232917))
shortest += g.shortest_path(vg.Point(1.7286467154371232,-3.3592566534232917), vg.Point(-6.453060574822895,-5.554029587082576))
shortest += g.shortest_path(vg.Point(-6.453060574822895,-5.554029587082576), vg.Point(7.548033057492154,-3.6395894800134774))
shortest += g.shortest_path(vg.Point(7.548033057492154,-3.6395894800134774), vg.Point(1.3847553205953984,-4.943156251673194))
shortest += g.shortest_path(vg.Point(1.3847553205953984,-4.943156251673194), vg.Point(7.54428351186684,-5.122805239294733))
shortest += g.shortest_path(vg.Point(7.54428351186684,-5.122805239294733), vg.Point(6.069523460454115,-4.654963174682476))
shortest += g.shortest_path(vg.Point(6.069523460454115,-4.654963174682476), vg.Point(-3.7547046958434,-5.922862243521676))
shortest += g.shortest_path(vg.Point(-3.7547046958434,-5.922862243521676), vg.Point(8.07226580148582,-3.7043831068229327))
shortest += g.shortest_path(vg.Point(8.07226580148582,-3.7043831068229327), vg.Point(3.268322380456488,6.87301494816695))
shortest += g.shortest_path(vg.Point(3.268322380456488,6.87301494816695), vg.Point(-0.23603546301427958,-1.6753606362682891))
shortest += g.shortest_path(vg.Point(-0.23603546301427958,-1.6753606362682891), vg.Point(5.7410221165283835,-5.033349148339862))
shortest += g.shortest_path(vg.Point(5.7410221165283835,-5.033349148339862), vg.Point(-3.572734367362853,7.552213061823806))
shortest += g.shortest_path(vg.Point(-3.572734367362853,7.552213061823806), vg.Point(4.81906153781483,1.9800634058196565))
shortest += g.shortest_path(vg.Point(4.81906153781483,1.9800634058196565), vg.Point(4.715317354682997,-3.8923377625585642))
shortest += g.shortest_path(vg.Point(4.715317354682997,-3.8923377625585642), vg.Point(4.748705754540919,-3.4749419274934694))
shortest += g.shortest_path(vg.Point(4.748705754540919,-3.4749419274934694), vg.Point(5.5460592904260055,6.6983154240782))
shortest += g.shortest_path(vg.Point(5.5460592904260055,6.6983154240782), vg.Point(-5.1248967023390675,4.5660839994153575))
shortest += g.shortest_path(vg.Point(-5.1248967023390675,4.5660839994153575), vg.Point(-5.392545551804345,4.952462262791909))
shortest += g.shortest_path(vg.Point(-5.392545551804345,4.952462262791909), vg.Point(-2.3217308730612407,-4.312281956042135))
shortest += g.shortest_path(vg.Point(-2.3217308730612407,-4.312281956042135), vg.Point(7.1765781232461565,8.292057771681467))
shortest += g.shortest_path(vg.Point(7.1765781232461565,8.292057771681467), vg.Point(3.777882182182938,1.5017283946377313))
shortest += g.shortest_path(vg.Point(3.777882182182938,1.5017283946377313), vg.Point(0.5183545338567797,6.284145566423535))
shortest += g.shortest_path(vg.Point(0.5183545338567797,6.284145566423535), vg.Point(6.092954859853264,-2.775568375975115))
shortest += g.shortest_path(vg.Point(6.092954859853264,-2.775568375975115), vg.Point(0.5311369877823493,8.12312296540589))
shortest += g.shortest_path(vg.Point(0.5311369877823493,8.12312296540589), vg.Point(-4.984831097840179,-1.316710012155605))
shortest += g.shortest_path(vg.Point(-4.984831097840179,-1.316710012155605), vg.Point(-4.044553473757543,5.295550268280642))
shortest += g.shortest_path(vg.Point(-4.044553473757543,5.295550268280642), vg.Point(-1.8287418950606478,-2.0036206511860257))
shortest += g.shortest_path(vg.Point(-1.8287418950606478,-2.0036206511860257), vg.Point(4.689812573707262,1.118966583506718))
shortest += g.shortest_path(vg.Point(4.689812573707262,1.118966583506718), vg.Point(-6.30809112233693,-3.423770818145659))
shortest += g.shortest_path(vg.Point(-6.30809112233693,-3.423770818145659), vg.Point(-4.716591528644418,-0.535560816076492))
shortest += g.shortest_path(vg.Point(-4.716591528644418,-0.535560816076492), vg.Point(4.482578581463751,1.5283944740509536))
shortest += g.shortest_path(vg.Point(4.482578581463751,1.5283944740509536), vg.Point(0.4455311690117645,6.18488677664192))
shortest += g.shortest_path(vg.Point(0.4455311690117645,6.18488677664192), vg.Point(6.4849370898978025,-2.1915511517535906))
shortest += g.shortest_path(vg.Point(6.4849370898978025,-2.1915511517535906), vg.Point(-4.913418124436547,8.081318894398798))
shortest += g.shortest_path(vg.Point(-4.913418124436547,8.081318894398798), vg.Point(5.022656397893212,-2.7608289720228574))
shortest += g.shortest_path(vg.Point(5.022656397893212,-2.7608289720228574), vg.Point(1.8798535943447847,-0.6383019674365489))
shortest += g.shortest_path(vg.Point(1.8798535943447847,-0.6383019674365489), vg.Point(-4.479245268156541,5.078256011534479))
shortest += g.shortest_path(vg.Point(-4.479245268156541,5.078256011534479), vg.Point(6.344455275249538,1.329287354089785))
shortest += g.shortest_path(vg.Point(6.344455275249538,1.329287354089785), vg.Point(6.57261450653339,-5.301205210090737))
shortest += g.shortest_path(vg.Point(6.57261450653339,-5.301205210090737), vg.Point(7.63100896637573,1.738124725253063))
shortest += g.shortest_path(vg.Point(7.63100896637573,1.738124725253063), vg.Point(6.382889837024428,4.80635547189856))
shortest += g.shortest_path(vg.Point(6.382889837024428,4.80635547189856), vg.Point(1.7730736494644317,-2.741939970082505))
shortest += g.shortest_path(vg.Point(1.7730736494644317,-2.741939970082505), vg.Point(2.5744559952202186,0.30488052494965334))
shortest += g.shortest_path(vg.Point(2.5744559952202186,0.30488052494965334), vg.Point(6.427969280461144,-3.3622836044372106))
shortest += g.shortest_path(vg.Point(6.427969280461144,-3.3622836044372106), vg.Point(-6.057352547804057,-4.579248928098282))
shortest += g.shortest_path(vg.Point(-6.057352547804057,-4.579248928098282), vg.Point(-1.3002157590205492,6.368914071163556))
shortest += g.shortest_path(vg.Point(-1.3002157590205492,6.368914071163556), vg.Point(-1.287303589354793,1.1882800807818956))
shortest += g.shortest_path(vg.Point(-1.287303589354793,1.1882800807818956), vg.Point(-1.788658351617726,-0.12416042706065422))
shortest += g.shortest_path(vg.Point(-1.788658351617726,-0.12416042706065422), vg.Point(4.181030805569011,3.8092654021697934))
shortest += g.shortest_path(vg.Point(4.181030805569011,3.8092654021697934), vg.Point(0.5987523399188897,5.260960668076128))
shortest += g.shortest_path(vg.Point(0.5987523399188897,5.260960668076128), vg.Point(3.6562263543941365,-0.5658475093721069))
shortest += g.shortest_path(vg.Point(3.6562263543941365,-0.5658475093721069), vg.Point(5.693423870458452,0.4164082744067814))
shortest += g.shortest_path(vg.Point(5.693423870458452,0.4164082744067814), vg.Point(-4.321501660402658,-4.456941382383687))
shortest += g.shortest_path(vg.Point(-4.321501660402658,-4.456941382383687), vg.Point(-4.612247153395806,3.829994051433551))
shortest += g.shortest_path(vg.Point(-4.612247153395806,3.829994051433551), vg.Point(0.30136669473326805,-0.2156774475360601))
shortest += g.shortest_path(vg.Point(0.30136669473326805,-0.2156774475360601), vg.Point(7.538395029430419,6.086625229536261))
shortest += g.shortest_path(vg.Point(7.538395029430419,6.086625229536261), vg.Point(2.521044193397004,2.7833014013462796))
shortest += g.shortest_path(vg.Point(2.521044193397004,2.7833014013462796), vg.Point(-3.205820119702892,8.036067317096311))
shortest += g.shortest_path(vg.Point(-3.205820119702892,8.036067317096311), vg.Point(4.6964176150134245,-5.078331659520719))
shortest += g.shortest_path(vg.Point(4.6964176150134245,-5.078331659520719), vg.Point(-1.6280542910310398,7.990489676732845))
shortest += g.shortest_path(vg.Point(-1.6280542910310398,7.990489676732845), vg.Point(-2.6106312086221592,-4.490064377416797))
shortest += g.shortest_path(vg.Point(-2.6106312086221592,-4.490064377416797), vg.Point(6.170146848608327,2.6317971255677906))
shortest += g.shortest_path(vg.Point(6.170146848608327,2.6317971255677906), vg.Point(7.264489879066921,-1.7977843984976092))
shortest += g.shortest_path(vg.Point(7.264489879066921,-1.7977843984976092), vg.Point(3.6808085503323236,-1.4245323836814894))
shortest += g.shortest_path(vg.Point(3.6808085503323236,-1.4245323836814894), vg.Point(-4.918965105549663,8.29552691059055))
shortest += g.shortest_path(vg.Point(-4.918965105549663,8.29552691059055), vg.Point(-4.963389653872008,-4.542198610532715))
shortest += g.shortest_path(vg.Point(-4.963389653872008,-4.542198610532715), vg.Point(-1.8471622418386797,-2.620020574934525))
shortest += g.shortest_path(vg.Point(-1.8471622418386797,-2.620020574934525), vg.Point(-6.158494054910672,4.832306410269326))
shortest += g.shortest_path(vg.Point(-6.158494054910672,4.832306410269326), vg.Point(3.583623786998344,8.228503826577253))
shortest += g.shortest_path(vg.Point(3.583623786998344,8.228503826577253), vg.Point(5.856255988579135,-3.3541159123469337))
shortest += g.shortest_path(vg.Point(5.856255988579135,-3.3541159123469337), vg.Point(3.4250893144721433,7.447867490632302))
shortest += g.shortest_path(vg.Point(3.4250893144721433,7.447867490632302), vg.Point(0.6423588364572801,-2.472384061943537))
shortest += g.shortest_path(vg.Point(0.6423588364572801,-2.472384061943537), vg.Point(-4.1577442406987295,6.6662877272485455))
shortest += g.shortest_path(vg.Point(-4.1577442406987295,6.6662877272485455), vg.Point(-3.852268794082571,4.5634314155888385))
shortest += g.shortest_path(vg.Point(-3.852268794082571,4.5634314155888385), vg.Point(2.0452830203720405,7.186721143020939))
shortest += g.shortest_path(vg.Point(2.0452830203720405,7.186721143020939), vg.Point(3.250059279540161,6.753195379630367))
shortest += g.shortest_path(vg.Point(3.250059279540161,6.753195379630367), vg.Point(4.88257377272856,0.3825841113610746))
shortest += g.shortest_path(vg.Point(4.88257377272856,0.3825841113610746), vg.Point(-5.457943264046682,-1.733974330494143))
shortest += g.shortest_path(vg.Point(-5.457943264046682,-1.733974330494143), vg.Point(-0.23350260020815483,6.811155081017756))
shortest += g.shortest_path(vg.Point(-0.23350260020815483,6.811155081017756), vg.Point(-6.70295522422186,-4.945460151408957))
shortest += g.shortest_path(vg.Point(-6.70295522422186,-4.945460151408957), vg.Point(-4.783391721935423,3.857389962835348))
shortest += g.shortest_path(vg.Point(-4.783391721935423,3.857389962835348), vg.Point(-5.157610615246372,-2.277416451686563))
shortest += g.shortest_path(vg.Point(-5.157610615246372,-2.277416451686563), vg.Point(0.5990437760730734,4.196517832316757))
shortest += g.shortest_path(vg.Point(0.5990437760730734,4.196517832316757), vg.Point(-5.271233321976013,7.258607493577954))
shortest += g.shortest_path(vg.Point(-5.271233321976013,7.258607493577954), vg.Point(-3.720812228177616,-4.008457892597985))
shortest += g.shortest_path(vg.Point(-3.720812228177616,-4.008457892597985), vg.Point(4.666101135392579,-3.8678865237426545))
shortest += g.shortest_path(vg.Point(4.666101135392579,-3.8678865237426545), vg.Point(3.4244559446025162,1.6371673455419966))
shortest += g.shortest_path(vg.Point(3.4244559446025162,1.6371673455419966), vg.Point(7.6506160594950545,-4.779703964356228))
shortest += g.shortest_path(vg.Point(7.6506160594950545,-4.779703964356228), vg.Point(1.4229928081412178,4.2364709505478215))
shortest += g.shortest_path(vg.Point(1.4229928081412178,4.2364709505478215), vg.Point(3.181575868501068,4.040065465836787))
shortest += g.shortest_path(vg.Point(3.181575868501068,4.040065465836787), vg.Point(1.9249637997589817,5.686935126665827))
shortest += g.shortest_path(vg.Point(1.9249637997589817,5.686935126665827), vg.Point(-1.9749657545824375,0.326640873787011))
shortest += g.shortest_path(vg.Point(-1.9749657545824375,0.326640873787011), vg.Point(5.199848659454887,7.798093200242936))
shortest += g.shortest_path(vg.Point(5.199848659454887,7.798093200242936), vg.Point(4.834143842795577,2.718471268911519))
shortest += g.shortest_path(vg.Point(4.834143842795577,2.718471268911519), vg.Point(2.0081887795594477,-0.6073054412326799))
shortest += g.shortest_path(vg.Point(2.0081887795594477,-0.6073054412326799), vg.Point(2.1862617965660016,-5.0823412265754335))
shortest += g.shortest_path(vg.Point(2.1862617965660016,-5.0823412265754335), vg.Point(-1.2114554132257362,5.656851204119694))
shortest += g.shortest_path(vg.Point(-1.2114554132257362,5.656851204119694), vg.Point(-1.1193292419264615,-0.8911270167957932))
shortest += g.shortest_path(vg.Point(-1.1193292419264615,-0.8911270167957932), vg.Point(-5.785627369121695,5.7760457258746625))
shortest += g.shortest_path(vg.Point(-5.785627369121695,5.7760457258746625), vg.Point(5.75854238068434,-1.8348631409702518))
shortest += g.shortest_path(vg.Point(5.75854238068434,-1.8348631409702518), vg.Point(-6.237067067904015,7.272881247133454))
shortest += g.shortest_path(vg.Point(-6.237067067904015,7.272881247133454), vg.Point(-4.466869370955683,5.402656452545206))
shortest += g.shortest_path(vg.Point(-4.466869370955683,5.402656452545206), vg.Point(6.500795366272993,1.7985609780485925))
shortest += g.shortest_path(vg.Point(6.500795366272993,1.7985609780485925), vg.Point(-0.013224632648134893,5.624906533035998))
shortest += g.shortest_path(vg.Point(-0.013224632648134893,5.624906533035998), vg.Point(5.713519862200783,0.0371583235365609))
shortest += g.shortest_path(vg.Point(5.713519862200783,0.0371583235365609), vg.Point(1.4393245345179375,-3.8847578742816524))
shortest += g.shortest_path(vg.Point(1.4393245345179375,-3.8847578742816524), vg.Point(7.816150056355288,-3.222837298213615))
shortest += g.shortest_path(vg.Point(7.816150056355288,-3.222837298213615), vg.Point(4.639322038735824,7.180157818416753))
shortest += g.shortest_path(vg.Point(4.639322038735824,7.180157818416753), vg.Point(5.8158669552286275,0.740779373737146))
shortest += g.shortest_path(vg.Point(5.8158669552286275,0.740779373737146), vg.Point(7.359462888550909,6.147223823325191))
shortest += g.shortest_path(vg.Point(7.359462888550909,6.147223823325191), vg.Point(4.120537903753147,7.732517123456904))
shortest += g.shortest_path(vg.Point(4.120537903753147,7.732517123456904), vg.Point(-1.7807428254991935,7.519039005570991))
shortest += g.shortest_path(vg.Point(-1.7807428254991935,7.519039005570991), vg.Point(0.7465081534747418,5.6101876625716285))
shortest += g.shortest_path(vg.Point(0.7465081534747418,5.6101876625716285), vg.Point(0.9838156241238671,-2.770041751461609))
shortest += g.shortest_path(vg.Point(0.9838156241238671,-2.770041751461609), vg.Point(-6.0653733562613965,6.769937731630743))
shortest += g.shortest_path(vg.Point(-6.0653733562613965,6.769937731630743), vg.Point(-1.4349215484028237,3.8455290371136064))
shortest += g.shortest_path(vg.Point(-1.4349215484028237,3.8455290371136064), vg.Point(-2.5786651848935787,5.189917289530594))
shortest += g.shortest_path(vg.Point(-2.5786651848935787,5.189917289530594), vg.Point(-3.6402659875882186,1.5738600985019806))
shortest += g.shortest_path(vg.Point(-3.6402659875882186,1.5738600985019806), vg.Point(6.202416057614024,8.467895789094996))
shortest += g.shortest_path(vg.Point(6.202416057614024,8.467895789094996), vg.Point(-3.2483339919722307,5.8841156676234645))
shortest += g.shortest_path(vg.Point(-3.2483339919722307,5.8841156676234645), vg.Point(5.802779991995073,-5.932824013422136))
shortest += g.shortest_path(vg.Point(5.802779991995073,-5.932824013422136), vg.Point(-4.133657146733643,-1.8549606621594004))
shortest += g.shortest_path(vg.Point(-4.133657146733643,-1.8549606621594004), vg.Point(-5.444545296305635,7.569995420942168))
shortest += g.shortest_path(vg.Point(-5.444545296305635,7.569995420942168), vg.Point(2.5448715227696512,-1.0369343760909198))
shortest += g.shortest_path(vg.Point(2.5448715227696512,-1.0369343760909198), vg.Point(3.6810534482179387,-3.8394939980312888))
shortest += g.shortest_path(vg.Point(3.6810534482179387,-3.8394939980312888), vg.Point(8.102502422790518,-0.6675134448085274))
shortest += g.shortest_path(vg.Point(8.102502422790518,-0.6675134448085274), vg.Point(4.373665577804144,8.402818394485003))
shortest += g.shortest_path(vg.Point(4.373665577804144,8.402818394485003), vg.Point(-0.7094573941126807,6.887803345371094))
shortest += g.shortest_path(vg.Point(-0.7094573941126807,6.887803345371094), vg.Point(0.4651044887588478,7.298428603670792))
shortest += g.shortest_path(vg.Point(0.4651044887588478,7.298428603670792), vg.Point(5.560719660822557,-4.982407345591296))
shortest += g.shortest_path(vg.Point(5.560719660822557,-4.982407345591296), vg.Point(5.394099386231296,-1.6388855875544195))
shortest += g.shortest_path(vg.Point(5.394099386231296,-1.6388855875544195), vg.Point(7.5298057430505105,-1.9892292663300681))
shortest += g.shortest_path(vg.Point(7.5298057430505105,-1.9892292663300681), vg.Point(6.892855272674088,-0.5937947418630616))
shortest += g.shortest_path(vg.Point(6.892855272674088,-0.5937947418630616), vg.Point(-6.154590237386031,-2.6310167838522003))
shortest += g.shortest_path(vg.Point(-6.154590237386031,-2.6310167838522003), vg.Point(2.691340569100814,-5.818584948025541))
shortest += g.shortest_path(vg.Point(2.691340569100814,-5.818584948025541), vg.Point(7.117457396849843,-0.22083786570504493))
shortest += g.shortest_path(vg.Point(7.117457396849843,-0.22083786570504493), vg.Point(7.457977218246763,-1.0298937969363626))
shortest += g.shortest_path(vg.Point(7.457977218246763,-1.0298937969363626), vg.Point(7.84815943007754,-0.565157557641359))
shortest += g.shortest_path(vg.Point(7.84815943007754,-0.565157557641359), vg.Point(-1.8667196752569009,0.0939600881616478))
shortest += g.shortest_path(vg.Point(-1.8667196752569009,0.0939600881616478), vg.Point(1.0590787635810806,3.0536227066617885))
shortest += g.shortest_path(vg.Point(1.0590787635810806,3.0536227066617885), vg.Point(3.6587627404941427,-5.768358805694836))
shortest += g.shortest_path(vg.Point(3.6587627404941427,-5.768358805694836), vg.Point(-2.10145664290307,4.634746937350806))
shortest += g.shortest_path(vg.Point(-2.10145664290307,4.634746937350806), vg.Point(5.583603367661769,2.521712903685027))
shortest += g.shortest_path(vg.Point(5.583603367661769,2.521712903685027), vg.Point(1.4072099810963001,7.520163010483652))
shortest += g.shortest_path(vg.Point(1.4072099810963001,7.520163010483652), vg.Point(-2.971869498676368,7.729122941168884))
shortest += g.shortest_path(vg.Point(-2.971869498676368,7.729122941168884), vg.Point(-6.0069317705424945,-4.863131398177553))
shortest += g.shortest_path(vg.Point(-6.0069317705424945,-4.863131398177553), vg.Point(-6.351229972392486,7.9830178399070055))
shortest += g.shortest_path(vg.Point(-6.351229972392486,7.9830178399070055), vg.Point(-5.563016904846581,-5.81299682621115))
shortest += g.shortest_path(vg.Point(-5.563016904846581,-5.81299682621115), vg.Point(0.827385242329937,-3.154107603214514))
shortest += g.shortest_path(vg.Point(0.827385242329937,-3.154107603214514), vg.Point(-6.454074575246995,-1.412478324306166))
shortest += g.shortest_path(vg.Point(-6.454074575246995,-1.412478324306166), vg.Point(3.3014635550083833,-5.215244328698136))
shortest += g.shortest_path(vg.Point(3.3014635550083833,-5.215244328698136), vg.Point(3.7960877902297367,-2.15815013489891))
shortest += g.shortest_path(vg.Point(3.7960877902297367,-2.15815013489891), vg.Point(7.011374681645717,-1.1012226161701335))
shortest += g.shortest_path(vg.Point(7.011374681645717,-1.1012226161701335), vg.Point(-4.738958255779945,-2.546487163278671))
shortest += g.shortest_path(vg.Point(-4.738958255779945,-2.546487163278671), vg.Point(-5.486809608364741,4.169259889991571))
shortest += g.shortest_path(vg.Point(-5.486809608364741,4.169259889991571), vg.Point(-5.443956747210483,-1.8446484911258532))
shortest += g.shortest_path(vg.Point(-5.443956747210483,-1.8446484911258532), vg.Point(-1.8165506695596862,6.00695261255007))
shortest += g.shortest_path(vg.Point(-1.8165506695596862,6.00695261255007), vg.Point(-6.022637515411948,-3.3688760748678646))
shortest += g.shortest_path(vg.Point(-6.022637515411948,-3.3688760748678646), vg.Point(-4.379908959188498,6.783101228645848))
shortest += g.shortest_path(vg.Point(-4.379908959188498,6.783101228645848), vg.Point(-2.5401817105894935,4.9443233919488305))
shortest += g.shortest_path(vg.Point(-2.5401817105894935,4.9443233919488305), vg.Point(5.887459685678761,-3.8274345436115307))
shortest += g.shortest_path(vg.Point(5.887459685678761,-3.8274345436115307), vg.Point(2.4899979525665943,3.962096580823869))
shortest += g.shortest_path(vg.Point(2.4899979525665943,3.962096580823869), vg.Point(-1.104062437366955,1.0565026766515393))
shortest += g.shortest_path(vg.Point(-1.104062437366955,1.0565026766515393), vg.Point(8.102932964645166,6.715089326547218))
shortest += g.shortest_path(vg.Point(8.102932964645166,6.715089326547218), vg.Point(5.387731315433247,0.3844175532061831))
shortest += g.shortest_path(vg.Point(5.387731315433247,0.3844175532061831), vg.Point(6.3095116881784845,-3.8757009250238177))
shortest += g.shortest_path(vg.Point(6.3095116881784845,-3.8757009250238177), vg.Point(-0.1360746446263681,0.39868626045554567))
shortest += g.shortest_path(vg.Point(-0.1360746446263681,0.39868626045554567), vg.Point(-6.423973590069131,4.671708613156295))
shortest += g.shortest_path(vg.Point(-6.423973590069131,4.671708613156295), vg.Point(-0.5186710851469867,-2.5614658664604586))
shortest += g.shortest_path(vg.Point(-0.5186710851469867,-2.5614658664604586), vg.Point(5.294959936854654,0.7546653550183651))
shortest += g.shortest_path(vg.Point(5.294959936854654,0.7546653550183651), vg.Point(2.0669825304151823,-2.3607914760751645))
shortest += g.shortest_path(vg.Point(2.0669825304151823,-2.3607914760751645), vg.Point(3.517479038134561,-4.621464338760124))
shortest += g.shortest_path(vg.Point(3.517479038134561,-4.621464338760124), vg.Point(-5.0298384099569375,3.445242533020502))
shortest += g.shortest_path(vg.Point(-5.0298384099569375,3.445242533020502), vg.Point(6.46284337202088,3.761893483367337))
shortest += g.shortest_path(vg.Point(6.46284337202088,3.761893483367337), vg.Point(-1.6951278115382689,5.293052000422565))
shortest += g.shortest_path(vg.Point(-1.6951278115382689,5.293052000422565), vg.Point(-5.799402804024482,6.212834927246061))
shortest += g.shortest_path(vg.Point(-5.799402804024482,6.212834927246061), vg.Point(1.5557585479348477,-3.623292031988746))
shortest += g.shortest_path(vg.Point(1.5557585479348477,-3.623292031988746), vg.Point(3.3070237365924813,-2.7485282008738645))
shortest += g.shortest_path(vg.Point(3.3070237365924813,-2.7485282008738645), vg.Point(4.395076511229729,-3.669401081286677))
shortest += g.shortest_path(vg.Point(4.395076511229729,-3.669401081286677), vg.Point(-0.48958338397042755,0.060508866467598565))
shortest += g.shortest_path(vg.Point(-0.48958338397042755,0.060508866467598565), vg.Point(2.419845595634068,6.804867516083051))
shortest += g.shortest_path(vg.Point(2.419845595634068,6.804867516083051), vg.Point(-5.22287120953545,-0.23130559774757842))
shortest += g.shortest_path(vg.Point(-5.22287120953545,-0.23130559774757842), vg.Point(-6.4246591482969135,0.5663932732174883))
shortest += g.shortest_path(vg.Point(-6.4246591482969135,0.5663932732174883), vg.Point(3.6424833348488166,-2.52891567034766))
shortest += g.shortest_path(vg.Point(3.6424833348488166,-2.52891567034766), vg.Point(-0.7111359405177549,6.103498593429385))
shortest += g.shortest_path(vg.Point(-0.7111359405177549,6.103498593429385), vg.Point(-2.2266208956569526,-4.502957790915962))
shortest += g.shortest_path(vg.Point(-2.2266208956569526,-4.502957790915962), vg.Point(-5.876941749936402,-0.6009145815962436))
shortest += g.shortest_path(vg.Point(-5.876941749936402,-0.6009145815962436), vg.Point(-4.84790355350174,5.626889726334723))
shortest += g.shortest_path(vg.Point(-4.84790355350174,5.626889726334723), vg.Point(-6.653788481621504,8.164765214471114))
shortest += g.shortest_path(vg.Point(-6.653788481621504,8.164765214471114), vg.Point(-1.3447074305066922,-0.19506457753300754))
shortest += g.shortest_path(vg.Point(-1.3447074305066922,-0.19506457753300754), vg.Point(-0.1656078456355523,2.818503665538999))
shortest += g.shortest_path(vg.Point(-0.1656078456355523,2.818503665538999), vg.Point(1.6431483367636668,-3.177392859717861))
shortest += g.shortest_path(vg.Point(1.6431483367636668,-3.177392859717861), vg.Point(-3.0693554983542364,2.2671675897705965))
shortest += g.shortest_path(vg.Point(-3.0693554983542364,2.2671675897705965), vg.Point(2.7101872843437382,-0.502026206187649))
shortest += g.shortest_path(vg.Point(2.7101872843437382,-0.502026206187649), vg.Point(2.7268087606292672,-1.195518154433092))
shortest += g.shortest_path(vg.Point(2.7268087606292672,-1.195518154433092), vg.Point(0.46189235793172845,-0.13062547525178392))
shortest += g.shortest_path(vg.Point(0.46189235793172845,-0.13062547525178392), vg.Point(4.233569236084011,-0.1378887119870349))
shortest += g.shortest_path(vg.Point(4.233569236084011,-0.1378887119870349), vg.Point(-1.7478757011512593,-5.705510155138268))
shortest += g.shortest_path(vg.Point(-1.7478757011512593,-5.705510155138268), vg.Point(-1.8665782663581032,-0.07015785448803413))
shortest += g.shortest_path(vg.Point(-1.8665782663581032,-0.07015785448803413), vg.Point(-1.3524631269945777,3.5009702267499145))
shortest += g.shortest_path(vg.Point(-1.3524631269945777,3.5009702267499145), vg.Point(-2.2659052202810814,6.089162417087251))
shortest += g.shortest_path(vg.Point(-2.2659052202810814,6.089162417087251), vg.Point(-2.2742993837516465,0.37707441225235083))
shortest += g.shortest_path(vg.Point(-2.2742993837516465,0.37707441225235083), vg.Point(0.2110360615927771,-5.8205791832063785))
shortest += g.shortest_path(vg.Point(0.2110360615927771,-5.8205791832063785), vg.Point(4.583925243522908,0.7956846800927186))
shortest += g.shortest_path(vg.Point(4.583925243522908,0.7956846800927186), vg.Point(1.925971563899341,7.6632231145256755))
shortest += g.shortest_path(vg.Point(1.925971563899341,7.6632231145256755), vg.Point(-0.15198638131848963,-5.302377845270425))
shortest += g.shortest_path(vg.Point(-0.15198638131848963,-5.302377845270425), vg.Point(7.682093746471817,2.8165182708423613))
shortest += g.shortest_path(vg.Point(7.682093746471817,2.8165182708423613), vg.Point(6.9261338601069555,-1.623362689011926))
shortest += g.shortest_path(vg.Point(6.9261338601069555,-1.623362689011926), vg.Point(-1.3224051718684144,1.366424721704818))
shortest += g.shortest_path(vg.Point(-1.3224051718684144,1.366424721704818), vg.Point(2.791061376537047,-5.345422136580185))
shortest += g.shortest_path(vg.Point(2.791061376537047,-5.345422136580185), vg.Point(-6.5817539335544515,8.12093865213157))
shortest += g.shortest_path(vg.Point(-6.5817539335544515,8.12093865213157), vg.Point(-2.1603219296384557,7.254514065259621))
shortest += g.shortest_path(vg.Point(-2.1603219296384557,7.254514065259621), vg.Point(-4.701926507454818,5.395784134933753))
shortest += g.shortest_path(vg.Point(-4.701926507454818,5.395784134933753), vg.Point(3.023663721224806,-5.882755948386988))
shortest += g.shortest_path(vg.Point(3.023663721224806,-5.882755948386988), vg.Point(-0.6676162604919709,-4.648821506198292))
shortest += g.shortest_path(vg.Point(-0.6676162604919709,-4.648821506198292), vg.Point(5.916902808895823,-2.9973195793729577))
shortest += g.shortest_path(vg.Point(5.916902808895823,-2.9973195793729577), vg.Point(3.894963083883991,4.419685206209317))
print shortest