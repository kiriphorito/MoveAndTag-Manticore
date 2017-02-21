#!/usr/bin/python
# coding: UTF-8
import matplotlib.pyplot as plt
from ast import literal_eval


def drawPolygon(points):
    polygon = plt.Polygon(points,facecolor='none')
    plt.gca().add_patch(polygon)

def drawPolygons(polygons):
    try:
        for xs in polygons:
            drawPolygon(xs)
    except ValueError:
        print ("no polygons specified")


def drawLine(points):
    line = plt.Polygon(points, closed=None, fill=None, edgecolor='r')
    plt.gca().add_patch(line)

#def drawLines(lines):
#    for xs in lines:
#        drawLine(xs)

def drawRobots(robots):
    for (x,y) in robots:
        plt.plot(x,y,".")

def samLines(coords):
    for lines in coords:
        print lines
        prex = "f"
        prey = "f"
        for (x,y) in lines:
            print (x,y)
            if prex == "f":
                prex = x
                prey = y
            else:
                line = plt.Polygon([(prex,prey),(x,y)], closed=None, fill=None)
                plt.gca().add_patch(line)
                prex = x
                prey = y
                print("connecting",(x,y),"to",(prex,prey))
#plt.plot((0,1),(1,2))


plt.axes()

samLines([[(2.2073865752517436, -1.889047496129887), (2.0669825304151823, -2.3607914760751645), (1.7474003547927728, -1.2477990406749173), (2.7268087606292672, -1.195518154433092), (3.3070237365924813, -2.7485282008738645), (3.996028251723197, -2.2093291846560725), (4.715317354682997, -3.8923377625585642), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (1.0590787635810806, 3.0536227066617885)],[(2.0669825304151823, -2.3607914760751645), (1.7730736494644317, -2.741939970082505), (1.6431483367636668, -3.177392859717861), (0.9838156241238671, -2.770041751461609), (0.6423588364572801, -2.472384061943537), (1.3161824199053331, -2.9449129674287895), (3.6810534482179387, -3.8394939980312888), (5.916902808895823, -2.9973195793729577), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (0.7514081303472473, 3.0733529751116455)],[(1.7730736494644317, -2.741939970082505), (1.7286467154371232, -3.3592566534232917), (1.5557585479348477, -3.623292031988746), (3.6424833348488166, -2.52891567034766), (3.517479038134561, -4.621464338760124), (5.856255988579135, -3.3541159123469337), (7.84815943007754, -0.565157557641359)],[(1.7474003547927728, -1.2477990406749173), (2.052386802076673, -0.8335145901272166), (1.8798535943447847, -0.6383019674365489), (2.0081887795594477, -0.6073054412326799), (0.5267146642115579, -0.4931108621138689), (0.30136669473326805, -0.2156774475360601), (1.9301998869031127, -0.25379933613156713), (4.154214433161562, 1.2618722266885047), (2.727656395595662, 0.5558178088987102), (1.802666880196373, 2.3290618407232597), (1.962511182428952, 3.827257744484897)],[(1.6431483367636668, -3.177392859717861), (1.4393245345179375, -3.8847578742816524), (1.72649613225727, -4.0856560752233335), (1.3847553205953984, -4.943156251673194), (3.3014635550083833, -5.215244328698136), (0.47907934816965103, -5.941444685700426), (-2.2266208956569526, -4.502957790915962), (-3.8010223726087475, -4.555270651862383), (-4.963389653872008, -4.542198610532715)],[(1.7286467154371232, -3.3592566534232917), (0.827385242329937, -3.154107603214514), (1.72649613225727, -4.0856560752233335), (2.1862617965660016, -5.0823412265754335), (2.691340569100814, -5.818584948025541), (0.47907934816965103, -5.941444685700426), (-2.3217308730612407, -4.312281956042135), (-1.8695413496807514, -3.8239034658823434), (-0.1292924782198224, -2.838239291135715), (1.3161824199053331, -2.9449129674287895), (4.200548350375849, -2.641071467236391), (5.147596690099898, -2.3199804486358633), (8.102502422790518, -0.6675134448085274)],[(1.8798535943447847, -0.6383019674365489), (2.5448715227696512, -1.0369343760909198), (2.5744559952202186, 0.30488052494965334), (1.9301998869031127, -0.25379933613156713), (-0.0246174585342621, 0.16891705339831045), (-0.1360746446263681, 0.39868626045554567), (-0.0246174585342621, 0.16891705339831045), (1.9301998869031127, -0.25379933613156713), (4.689812573707262, 1.118966583506718), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.1134161543212775, 4.140673372034827), (2.4899979525665943, 3.962096580823869)],[(2.7268087606292672, -1.195518154433092), (2.7101872843437382, -0.502026206187649), (3.6808085503323236, -1.4245323836814894), (4.200548350375849, -2.641071467236391), (4.667157304771778, -2.534219407619858), (4.200548350375849, -2.641071467236391), (3.558366313174796, -0.7469747877882957), (5.1313719740223664, 0.8422555891368066), (5.294959936854654, 0.7546653550183651), (4.925027685632841, 2.8299240979099256), (4.659784708236221, 3.526482298351895)],[(0.9838156241238671, -2.770041751461609), (-0.5186710851469867, -2.5614658664604586), (-1.8695413496807514, -3.8239034658823434), (-1.3767092623074424, -4.6940279016128095), (-0.6676162604919709, -4.648821506198292), (-1.3767092623074424, -4.6940279016128095), (-1.8695413496807514, -3.8239034658823434), (-1.100949608925354, -2.013056748416814), (-1.492534431882603, -1.4919471569285974), (-1.788658351617726, -0.12416042706065422), (-1.8985138332922995, 0.07593614482383877), (-3.888733837631598, 0.27348192262654597), (-4.984831097840179, -1.316710012155605)],[(1.5557585479348477, -3.623292031988746), (2.791061376537047, -5.345422136580185), (0.47907934816965103, -5.941444685700426), (0.2110360615927771, -5.8205791832063785), (0.47907934816965103, -5.941444685700426), (5.560719660822557, -4.982407345591296), (-0.1292924782198224, -2.838239291135715), (-2.194832499996357, -2.9093938616851127), (-3.189942502166012, -2.8106209727837586), (-5.157610615246372, -2.277416451686563)],[(2.0081887795594477, -0.6073054412326799), (0.46189235793172845, -0.13062547525178392), (-0.48958338397042755, 0.060508866467598565), (1.9301998869031127, -0.25379933613156713), (4.482578581463751, 1.5283944740509536), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.181575868501068, 4.040065465836787)],[(1.4393245345179375, -3.8847578742816524), (1.72649613225727, -4.0856560752233335), (1.3492037839001179, -5.448612598327115), (0.47907934816965103, -5.941444685700426), (-0.15198638131848963, -5.302377845270425), (0.47907934816965103, -5.941444685700426), (3.023663721224806, -5.882755948386988), (0.47907934816965103, -5.941444685700426), (-2.6106312086221592, -4.490064377416797), (-4.295555878780673, -2.6173757601633265), (-5.443956747210483, -1.8446484911258532)],[(0.827385242329937, -3.154107603214514), (-0.23603546301427958, -1.6753606362682891), (-0.7952159573962381, -2.2087085852574506), (-1.8471622418386797, -2.620020574934525), (-1.100949608925354, -2.013056748416814), (-1.492534431882603, -1.4919471569285974), (-1.8665782663581032, -0.07015785448803413), (-1.8985138332922995, 0.07593614482383877), (-3.888733837631598, 0.27348192262654597), (-5.457943264046682, -1.733974330494143)],[(2.5448715227696512, -1.0369343760909198), (3.6562263543941365, -0.5658475093721069), (4.88257377272856, 0.3825841113610746), (5.1313719740223664, 0.8422555891368066), (5.713519862200783, 0.0371583235365609), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (4.181030805569011, 3.8092654021697934)],[(2.7101872843437382, -0.502026206187649), (4.233569236084011, -0.1378887119870349), (3.4244559446025162, 1.6371673455419966), (4.027309841872726, 2.225932179379771), (6.170146848608327, 2.6317971255677906)],[(3.3070237365924813, -2.7485282008738645), (3.7960877902297367, -2.15815013489891), (4.395076511229729, -3.669401081286677), (6.092954859853264, -2.775568375975115), (7.63100896637573, 1.738124725253063)],[(0.6423588364572801, -2.472384061943537), (-0.7952159573962381, -2.2087085852574506), (-1.492534431882603, -1.4919471569285974), (-1.1193292419264615, -0.8911270167957932), (-1.8667196752569009, 0.0939600881616478), (-3.888733837631598, 0.27348192262654597), (-5.22287120953545, -0.23130559774757842)],[(3.6424833348488166, -2.52891567034766), (4.200548350375849, -2.641071467236391), (5.022656397893212, -2.7608289720228574), (5.887459685678761, -3.8274345436115307), (6.720602350947466, -0.7307500717107629), (6.39951133234694, 0.21629826801328667), (4.302652030414358, 4.117179861754971)],[(0.5267146642115579, -0.4931108621138689), (-0.05901157522490006, -0.09731020795587667), (-1.3447074305066922, -0.19506457753300754), (-1.104062437366955, 1.0565026766515393), (-1.7680766092066984, 1.5690421156468752), (-1.3524631269945777, 3.5009702267499145)],[(1.3847553205953984, -4.943156251673194), (3.6587627404941427, -5.768358805694836), (5.7410221165283835, -5.033349148339862), (0.47907934816965103, -5.941444685700426), (-5.563016904846581, -5.81299682621115)],[(2.1862617965660016, -5.0823412265754335), (4.6964176150134245, -5.078331659520719), (5.802779991995073, -5.932824013422136), (0.47907934816965103, -5.941444685700426), (-6.0069317705424945, -4.863131398177553)],[(2.5744559952202186, 0.30488052494965334), (3.6142784115079385, 1.0183125665983537), (3.777882182182938, 1.5017283946377313), (3.6142784115079385, 1.0183125665983537), (2.727656395595662, 0.5558178088987102), (1.802666880196373, 2.3290618407232597), (2.521044193397004, 2.7833014013462796), (1.4229928081412178, 4.2364709505478215)],[(3.6808085503323236, -1.4245323836814894), (4.200548350375849, -2.641071467236391), (5.147596690099898, -2.3199804486358633), (5.394099386231296, -1.6388855875544195), (6.720602350947466, -0.7307500717107629), (6.39951133234694, 0.21629826801328667), (5.693423870458452, 0.4164082744067814), (7.682093746471817, 2.8165182708423613)],[(-0.5186710851469867, -2.5614658664604586), (-1.8287418950606478, -2.0036206511860257), (-2.194832499996357, -2.9093938616851127), (-3.189942502166012, -2.8106209727837586), (-3.276919255050708, -2.3398760145109065), (-6.233450770479729, -3.1119092663352483), (-6.022637515411948, -3.3688760748678646)],[(2.791061376537047, -5.345422136580185), (4.666101135392579, -3.8678865237426545), (6.069523460454115, -4.654963174682476), (0.47907934816965103, -5.941444685700426), (-6.057352547804057, -4.579248928098282)],[(0.46189235793172845, -0.13062547525178392), (-1.238311811529301, 0.7476069382605006), (-1.287303589354793, 1.1882800807818956), (-1.7680766092066984, 1.5690421156468752), (-1.1340020249118816, 4.501268133802937), (0.5990437760730734, 4.196517832316757)],[(-0.15198638131848963, -5.302377845270425), (-1.7478757011512593, -5.705510155138268), (-3.030985979887064, -5.606359093806035), (-6.453060574822895, -5.554029587082576)],[(-0.23603546301427958, -1.6753606362682891), (-0.7952159573962381, -2.2087085852574506), (-1.492534431882603, -1.4919471569285974), (-1.7919488323765025, -0.9304890692153593), (-1.8985138332922995, 0.07593614482383877), (-1.9749657545824375, 0.326640873787011), (-1.4349215484028237, 3.8455290371136064)],[(3.6562263543941365, -0.5658475093721069), (4.583925243522908, 0.7956846800927186), (5.1313719740223664, 0.8422555891368066), (5.8158669552286275, 0.740779373737146), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.894963083883991, 4.419685206209317)],[(4.233569236084011, -0.1378887119870349), (5.1313719740223664, 0.8422555891368066), (5.387731315433247, 0.3844175532061831), (4.81906153781483, 1.9800634058196565), (6.46284337202088, 3.761893483367337)],[(3.7960877902297367, -2.15815013489891), (4.748705754540919, -3.4749419274934694), (6.4849370898978025, -2.1915511517535906), (6.720602350947466, -0.7307500717107629), (6.382889837024428, 4.80635547189856)],[(3.996028251723197, -2.2093291846560725), (4.200548350375849, -2.641071467236391), (5.147596690099898, -2.3199804486358633), (5.75854238068434, -1.8348631409702518), (6.427969280461144, -3.3622836044372106), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (0.26424382924462364, 4.408587560180833)],[(3.6810534482179387, -3.8394939980312888), (6.3095116881784845, -3.8757009250238177), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (0.5987523399188897, 5.260960668076128)],[(3.517479038134561, -4.621464338760124), (6.57261450653339, -5.301205210090737), (-0.1292924782198224, -2.838239291135715), (-2.194832499996357, -2.9093938616851127), (-3.189942502166012, -2.8106209727837586), (-4.295555878780673, -2.6173757601633265), (-6.154590237386031, -2.6310167838522003)],[(0.30136669473326805, -0.2156774475360601), (-1.3224051718684144, 1.366424721704818), (-1.7680766092066984, 1.5690421156468752), (-1.6363879702350923, 4.2742016914298455)],[(3.3014635550083833, -5.215244328698136), (7.54428351186684, -5.122805239294733), (0.47907934816965103, -5.941444685700426), (-3.8010223726087475, -4.555270651862383), (-6.30809112233693, -3.423770818145659)],[(2.691340569100814, -5.818584948025541), (7.6506160594950545, -4.779703964356228), (0.47907934816965103, -5.941444685700426), (-6.70295522422186, -4.945460151408957)],[(-0.1360746446263681, 0.39868626045554567), (-2.2742993837516465, 0.37707441225235083), (-2.10145664290307, 4.634746937350806)],[(4.667157304771778, -2.534219407619858), (6.9261338601069555, -1.623362689011926), (6.720602350947466, -0.7307500717107629), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (1.9249637997589817, 5.686935126665827)],[(-0.6676162604919709, -4.648821506198292), (-1.3767092623074424, -4.6940279016128095), (-3.720812228177616, -4.008457892597985), (-4.295555878780673, -2.6173757601633265), (-5.876941749936402, -0.6009145815962436)],[(0.2110360615927771, -5.8205791832063785), (-3.7547046958434, -5.922862243521676), (-6.233450770479729, -3.1119092663352483), (-6.454074575246995, -1.412478324306166)],[(-0.48958338397042755, 0.060508866467598565), (-1.7680766092066984, 1.5690421156468752), (-1.1340020249118816, 4.501268133802937), (-0.15659335219319442, 4.289909939037998), (-0.1656078456355523, 2.818503665538999), (-0.15659335219319442, 4.289909939037998), (-1.1340020249118816, 4.501268133802937), (-3.134941746562891, 4.392880537243911)],[(3.023663721224806, -5.882755948386988), (7.548033057492154, -3.6395894800134774), (4.200548350375849, -2.641071467236391), (2.052386802076673, -0.8335145901272166), (-0.05901157522490006, -0.09731020795587667), (-3.888733837631598, 0.27348192262654597), (-5.265531967924365, -0.18741289286898621), (-6.4246591482969135, 0.5663932732174883)],[(-1.8471622418386797, -2.620020574934525), (-2.194832499996357, -2.9093938616851127), (-3.189942502166012, -2.8106209727837586), (-4.133657146733643, -1.8549606621594004), (-6.616019419392999, 0.6818317478052967)],[(4.88257377272856, 0.3825841113610746), (5.1313719740223664, 0.8422555891368066), (6.344455275249538, 1.329287354089785), (8.090128739860756, 4.604285943372348), (8.203694322656643, 6.01393230271367), (7.359462888550909, 6.147223823325191)],[(3.4244559446025162, 1.6371673455419966), (4.834143842795577, 2.718471268911519), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (0.2854132295569878, 5.243555165301994)],[(4.395076511229729, -3.669401081286677), (7.758024410082515, -3.9166527369821686), (6.720602350947466, -0.7307500717107629), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (0.7465081534747418, 5.6101876625716285)],[(-1.1193292419264615, -0.8911270167957932), (-1.8985138332922995, 0.07593614482383877), (-3.8098482367621416, 0.7147790107512577), (-2.8928868598181494, 1.3522478460536007), (-3.5175923907117252, 4.021420795366142), (-4.509270890662411, 4.150159883197155), (-5.0298384099569375, 3.445242533020502)],[(5.022656397893212, -2.7608289720228574), (7.264489879066921, -1.7977843984976092), (8.090128739860756, 4.604285943372348), (8.203694322656643, 6.01393230271367), (7.538395029430419, 6.086625229536261)],[(-1.3447074305066922, -0.19506457753300754), (-2.8928868598181494, 1.3522478460536007), (-3.6402659875882186, 1.5738600985019806), (-3.5175923907117252, 4.021420795366142), (-4.509270890662411, 4.150159883197155), (-4.612247153395806, 3.829994051433551)],[(3.6587627404941427, -5.768358805694836), (8.07226580148582, -3.7043831068229327), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (-0.013224632648134893, 5.624906533035998)],[(4.6964176150134245, -5.078331659520719), (7.816150056355288, -3.222837298213615), (8.203694322656643, 6.01393230271367), (6.1460075750426135, 6.889103856578155), (5.5460592904260055, 6.6983154240782)],[(3.777882182182938, 1.5017283946377313), (5.583603367661769, 2.521712903685027), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (3.250059279540161, 6.753195379630367)],[(5.394099386231296, -1.6388855875544195), (7.011374681645717, -1.1012226161701335), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (3.268322380456488, 6.87301494816695)],[(-1.8287418950606478, -2.0036206511860257), (-2.194832499996357, -2.9093938616851127), (-3.189942502166012, -2.8106209727837586), (-4.295555878780673, -2.6173757601633265), (-4.738958255779945, -2.546487163278671), (-6.7501060662258014, 2.4242810589578054), (-6.621366978394792, 3.4159595589084892), (-4.783391721935423, 3.857389962835348)],[(4.666101135392579, -3.8678865237426545), (7.5298057430505105, -1.9892292663300681), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (0.4455311690117645, 6.18488677664192)],[(-1.238311811529301, 0.7476069382605006), (-3.0693554983542364, 2.2671675897705965), (-2.5401817105894935, 4.9443233919488305)],[(-1.7478757011512593, -5.705510155138268), (-4.321501660402658, -4.456941382383687), (-3.8010223726087475, -4.555270651862383), (-3.987506726532953, -0.7216280795431058), (-3.888733837631598, 0.27348192262654597), (-2.8928868598181494, 1.3522478460536007), (-3.5175923907117252, 4.021420795366142), (-3.852268794082571, 4.5634314155888385)],[(-1.7919488323765025, -0.9304890692153593), (-1.8985138332922995, 0.07593614482383877), (-3.888733837631598, 0.27348192262654597), (-4.716591528644418, -0.535560816076492), (-2.8928868598181494, 1.3522478460536007), (-2.5786651848935787, 5.189917289530594)],[(4.583925243522908, 0.7956846800927186), (6.500795366272993, 1.7985609780485925), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (4.639322038735824, 7.180157818416753)],[(5.387731315433247, 0.3844175532061831), (6.39951133234694, 0.21629826801328667), (6.892855272674088, -0.5937947418630616), (8.090128739860756, 4.604285943372348), (8.203694322656643, 6.01393230271367), (8.102932964645166, 6.715089326547218)],[(4.748705754540919, -3.4749419274934694), (7.457977218246763, -1.0298937969363626), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (2.419845595634068, 6.804867516083051)],[(5.75854238068434, -1.8348631409702518), (6.720602350947466, -0.7307500717107629), (7.117457396849843, -0.22083786570504493), (8.090128739860756, 4.604285943372348), (8.203694322656643, 6.01393230271367), (4.973369588410079, 7.5255867963970475)],[(4.715317354682997, -3.8923377625585642), (7.418773235738816, -0.44990765706479596), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (0.5183545338567797, 6.284145566423535)],[(5.916902808895823, -2.9973195793729577), (6.720602350947466, -0.7307500717107629), (8.090128739860756, 4.604285943372348), (8.203694322656643, 6.01393230271367), (7.638112697692372, 7.546541304735052)],[(5.856255988579135, -3.3541159123469337), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (2.451543302495492, 5.0585482745559), (3.4250893144721433, 7.447867490632302)],[(4.154214433161562, 1.2618722266885047), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (2.0452830203720405, 7.186721143020939)],[(-2.2266208956569526, -4.502957790915962), (-5.265531967924365, -0.18741289286898621), (-6.7501060662258014, 2.4242810589578054), (-6.621366978394792, 3.4159595589084892), (-5.486809608364741, 4.169259889991571)],[(-2.3217308730612407, -4.312281956042135), (-1.100949608925354, -2.013056748416814), (-1.492534431882603, -1.4919471569285974), (-1.8985138332922995, 0.07593614482383877), (-3.5175923907117252, 4.021420795366142), (-5.1248967023390675, 4.5660839994153575)],[(4.689812573707262, 1.118966583506718), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (4.120537903753147, 7.732517123456904)],[(5.294959936854654, 0.7546653550183651), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (5.199848659454887, 7.798093200242936)],[(-1.788658351617726, -0.12416042706065422), (-1.7680766092066984, 1.5690421156468752), (-1.6951278115382689, 5.293052000422565)],[(5.560719660822557, -4.982407345591296), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (-1.2114554132257362, 5.656851204119694)],[(4.482578581463751, 1.5283944740509536), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (1.925971563899341, 7.6632231145256755)],[(-2.6106312086221592, -4.490064377416797), (-3.987506726532953, -0.7216280795431058), (-3.888733837631598, 0.27348192262654597), (-2.8928868598181494, 1.3522478460536007), (-3.5175923907117252, 4.021420795366142), (-3.768419738963783, 5.061463126225703)],[(-1.8665782663581032, -0.07015785448803413), (-1.8985138332922995, 0.07593614482383877), (-3.5175923907117252, 4.021420795366142), (-4.479245268156541, 5.078256011534479)],[(5.713519862200783, 0.0371583235365609), (8.090128739860756, 4.604285943372348), (8.203694322656643, 6.01393230271367), (7.1765781232461565, 8.292057771681467)],[(4.027309841872726, 2.225932179379771), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (1.4072099810963001, 7.520163010483652)],[(6.092954859853264, -2.775568375975115), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (-0.7111359405177549, 6.103498593429385)],[(-1.8667196752569009, 0.0939600881616478), (-4.044553473757543, 5.295550268280642)],[(5.887459685678761, -3.8274345436115307), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (0.8212165138124101, 7.303175605155082)],[(-1.104062437366955, 1.0565026766515393), (-1.7680766092066984, 1.5690421156468752), (-1.8165506695596862, 6.00695261255007)],[(5.7410221165283835, -5.033349148339862), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (-0.23350260020815483, 6.811155081017756)],[(5.802779991995073, -5.932824013422136), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (-1.3002157590205492, 6.368914071163556)],[(2.521044193397004, 2.7833014013462796), (0.4651044887588478, 7.298428603670792)],[(5.693423870458452, 0.4164082744067814), (8.090128739860756, 4.604285943372348), (8.203694322656643, 6.01393230271367), (6.202416057614024, 8.467895789094996)],[(-3.276919255050708, -2.3398760145109065), (-5.265531967924365, -0.18741289286898621), (-6.7501060662258014, 2.4242810589578054), (-6.621366978394792, 3.4159595589084892), (-5.392545551804345, 4.952462262791909)],[(6.069523460454115, -4.654963174682476), (8.090128739860756, 4.604285943372348), (8.203694322656643, 6.01393230271367), (7.566930030403939, 8.35536654882807)],[(-1.287303589354793, 1.1882800807818956), (-1.7680766092066984, 1.5690421156468752), (-2.2659052202810814, 6.089162417087251)],[(-3.030985979887064, -5.606359093806035), (-4.295555878780673, -2.6173757601633265), (-6.7501060662258014, 2.4242810589578054), (-6.621366978394792, 3.4159595589084892), (-6.423973590069131, 4.671708613156295)],[(-1.9749657545824375, 0.326640873787011), (-3.5175923907117252, 4.021420795366142), (-4.466869370955683, 5.402656452545206)],[(5.8158669552286275, 0.740779373737146), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (4.373665577804144, 8.402818394485003)],[(4.81906153781483, 1.9800634058196565), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (3.583623786998344, 8.228503826577253)],[(6.4849370898978025, -2.1915511517535906), (6.720602350947466, -0.7307500717107629), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (-0.7094573941126807, 6.887803345371094)],[(6.427969280461144, -3.3622836044372106), (6.720602350947466, -0.7307500717107629), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (2.451543302495492, 5.0585482745559), (0.5311369877823493, 8.12312296540589)],[(6.3095116881784845, -3.8757009250238177), (4.200548350375849, -2.641071467236391), (2.1415580816680513, 0.7236093365871203), (1.1641494089493643, 0.9349675313520591), (-1.1340020249118816, 4.501268133802937), (-3.2483339919722307, 5.8841156676234645)],[(6.57261450653339, -5.301205210090737), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (-2.1055915905313096, 7.17999624927331)],[(-1.3224051718684144, 1.366424721704818), (-1.7680766092066984, 1.5690421156468752), (-3.5175923907117252, 4.021420795366142), (-4.701926507454818, 5.395784134933753)],[(7.54428351186684, -5.122805239294733), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (-1.7807428254991935, 7.519039005570991)],[(7.6506160594950545, -4.779703964356228), (4.200548350375849, -2.641071467236391), (1.802666880196373, 2.3290618407232597), (1.5868657984792418, 2.8897848767894345), (-2.1603219296384557, 7.254514065259621)],[(-2.2742993837516465, 0.37707441225235083), (-3.5175923907117252, 4.021420795366142), (-4.84790355350174, 5.626889726334723)],[(6.9261338601069555, -1.623362689011926), (6.720602350947466, -0.7307500717107629), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (0.25492505294213214, 6.481748034941264), (-1.6280542910310398, 7.990489676732845)],[(-3.720812228177616, -4.008457892597985), (-4.295555878780673, -2.6173757601633265), (-6.7501060662258014, 2.4242810589578054), (-6.621366978394792, 3.4159595589084892), (-6.158494054910672, 4.832306410269326)],[(-3.7547046958434, -5.922862243521676), (-3.8010223726087475, -4.555270651862383), (-4.295555878780673, -2.6173757601633265), (-6.7501060662258014, 2.4242810589578054), (-6.621366978394792, 3.4159595589084892), (-5.785627369121695, 5.7760457258746625)],[(-0.1656078456355523, 2.818503665538999), (-0.15659335219319442, 4.289909939037998), (-3.5164396704835954, 6.373383887540029)],[(7.548033057492154, -3.6395894800134774), (4.200548350375849, -2.641071467236391), (2.1415580816680513, 0.7236093365871203), (1.1641494089493643, 0.9349675313520591), (-1.1340020249118816, 4.501268133802937), (-3.3165405488513895, 6.68797610264568)],[(-4.133657146733643, -1.8549606621594004), (-5.265531967924365, -0.18741289286898621), (-6.7501060662258014, 2.4242810589578054), (-6.621366978394792, 3.4159595589084892), (-5.799402804024482, 6.212834927246061)],[(6.344455275249538, 1.329287354089785), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (-2.971869498676368, 7.729122941168884)],[(4.834143842795577, 2.718471268911519), (4.925027685632841, 2.8299240979099256), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (-3.205820119702892, 8.036067317096311)],[(7.758024410082515, -3.9166527369821686), (4.200548350375849, -2.641071467236391), (2.1415580816680513, 0.7236093365871203), (1.1641494089493643, 0.9349675313520591), (-1.1340020249118816, 4.501268133802937), (-4.1577442406987295, 6.6662877272485455)],[(-3.8098482367621416, 0.7147790107512577), (-2.8928868598181494, 1.3522478460536007), (-3.5175923907117252, 4.021420795366142), (-4.379908959188498, 6.783101228645848)],[(7.264489879066921, -1.7977843984976092), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (-3.572734367362853, 7.552213061823806)],[(-3.6402659875882186, 1.5738600985019806), (-3.5175923907117252, 4.021420795366142), (-6.343762246568933, 6.482770565702903)],[(8.07226580148582, -3.7043831068229327), (4.200548350375849, -2.641071467236391), (2.052386802076673, -0.8335145901272166), (-0.0246174585342621, 0.16891705339831045), (-3.5175923907117252, 4.021420795366142), (-5.271233321976013, 7.258607493577954)],[(7.816150056355288, -3.222837298213615), (4.200548350375849, -2.641071467236391), (2.1415580816680513, 0.7236093365871203), (1.1641494089493643, 0.9349675313520591), (-1.1340020249118816, 4.501268133802937), (-4.913418124436547, 8.081318894398798)],[(5.583603367661769, 2.521712903685027), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (-4.918965105549663, 8.29552691059055)],[(7.011374681645717, -1.1012226161701335), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (-5.444545296305635, 7.569995420942168)],[(-4.738958255779945, -2.546487163278671), (-6.7501060662258014, 2.4242810589578054), (-6.621366978394792, 3.4159595589084892), (-6.0653733562613965, 6.769937731630743)],[(7.5298057430505105, -1.9892292663300681), (6.39951133234694, 0.21629826801328667), (4.462532927933195, 3.7165461138222016), (3.0995836907682035, 4.296942303487298), (-5.4248143794161745, 8.136563438916923)],[(-3.0693554983542364, 2.2671675897705965), (-3.5175923907117252, 4.021420795366142), (-6.237067067904015, 7.272881247133454)],[(-4.321501660402658, -4.456941382383687), (-6.233450770479729, -3.1119092663352483), (-6.7501060662258014, 2.4242810589578054), (-6.351229972392486, 7.9830178399070055)],[(-4.716591528644418, -0.535560816076492), (-5.265531967924365, -0.18741289286898621), (-6.7501060662258014, 2.4242810589578054), (-6.5817539335544515, 8.12093865213157)],[(6.500795366272993, 1.7985609780485925), (4.462532927933195, 3.7165461138222016), (-6.653788481621504, 8.164765214471114)]]



)

fileName = '10.txt' #change here per question
checkState = 0
robots = []
polygons = []
with open(fileName,'r') as input: #reading txt file and drawing map
    for line in input:
        if 'Robots' in line:
            checkState = 1
            continue
        if 'Polygons' in line:
            checkState = 2
            continue
        if checkState == 1:
            line = line.strip('\n')
            lenstr = len(line)
            if lenstr != 0:
                robots.append(line)
        if checkState == 2:
            line = line.strip('\n')
            lenstr = len(line)
            if lenstr != 0:
                polygons.append(line)

robotpoints = list(literal_eval(robots[0]))
print robotpoints
drawRobots(robotpoints)

polygonpoints = list(literal_eval(polygons[0]))
print polygonpoints
drawPolygons(polygonpoints)

plt.axis('scaled')
plt.show()
