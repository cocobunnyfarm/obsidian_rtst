
# Boson 22 for Orin
CTIB-66448G REV B
PN: NGX021
SN: 1572


> [!Important] Warning
> CTIB-6644**7G** REV B '7G' 제품도 있으니 주의. 내가 가지고 있는 제품은 8G

https://connecttech.com/product/boson-22-carrier-board-for-nvidia-jetson-orin-nano/


# Products Received

## CBG408 (miniFIT-JR)
https://www.wdlsystems.com/CTI-Power-Input-Cable


# Ports
![[Screenshot_20260128_115303.png]]
![[Screenshot_20260128_115607.png]]
# Need to buy & Tools needed

## Storage

M2 M-Key 2280 SSD

https://search.danawa.com/dsearch.php?query=m.2+2280


## Wire cutter
- wire strippers 


## USB C TO C 데이터 케이블
https://www.coupang.com/vp/products/8087835532?itemId=22831071578&vendorItemId=91700276785&q=USB+C+TO+C+%EB%8D%B0%EC%9D%B4%ED%84%B0+%EC%BC%80%EC%9D%B4%EB%B8%94&searchId=ef54ed7c3163369&sourceType=search&itemsCount=36&searchRank=1&rank=1&traceId=mkxl7cdv

## DC 변환

- **터미널 블록 to 5.5x2.5mm DC 플러그 변환 잭 암**
		https://www.coupang.com/vp/products/7327567562

## 드라이버

그냥 키트로 사기 (**Phillips PH00, Torx T8H 포함**)

https://www.coupang.com/vp/products/6885610059?itemId=16514380181&vendorItemId=85156424629&sourceType=srp_product_ads&clickEventId=558dd8b0-fc09-11f0-9caf-bff86780bc17&korePlacement=15&koreSubPlacement=1&clickEventId=558dd8b0-fc09-11f0-9caf-bff86780bc17&korePlacement=15&koreSubPlacement=1&traceId=mkxkxrt1


screw 종류
https://auvidea.eu/product/leafspring-70696/?utm_source=chatgpt.com

필요한 드라이버:
Torx T8 (TX8) driver
https://www.coupang.com/vp/products/7761874811?itemId=20933124376&vendorItemId=87999829124&q=TX8+driver&searchId=fa33b14215613028&sourceType=search&itemsCount=36&searchRank=0&rank=0&traceId=mkxkawox

- **Torx T8H (Security Torx T8):** This is listed as `T8H` in the third row and is the correct size for the fan/heatsink screws on your Jetson module. The 'H' indicates it is the security version with a hole in the center, which is compatible with standard T8 screws.
- **Phillips PH00:** This is listed as `PH00` in the first row and is the correct size for the tiny M.2 SSD screw on your carrier board.


--------

- **The Power Adapter:** Your `EA1951E-190` (19V).
- **The Connector Cable:** Your `CBG408` (4-pin Mini-Fit Jr. with 2 Red and 2 Black wires).
- **The Bridge Part:** A **Female DC Power Jack to 2-Pin Terminal Block (5.5mm x 2.5mm)**.
    - _Critical:_ Ensure it is the **2.5mm** version (standard for high-power laptops and Jetson supplies), as the 2.1mm version will not fit your plug.
- **A Small Screwdriver:** Usually a small flat-head or Phillips, often included with the adapter.




# How to create a custom connector

## Recommendation for You (Given Your Use Case)

Given you’re doing Jetson / robotics power:

### Do this:

➡️ Buy a **5.5×2.5mm female DC jack to screw terminal adapter**  
➡️ Parallel red + red, black + black into it

This gives you:

- Proper current sharing
    
- No soldering
    
- Easily reversible
    
- Electrically correct
    

If this is for production or a lab you care about cleanliness:  
➡️ Order the Connect Tech power kit  
or  
➡️ Have a custom cable made

-------



Step-by-Step Assembly

Step A: Prepare the CBG408 Wires

1. **Strip the Ends:** If the wires are not already bare, use wire strippers (or carefully use scissors) to remove about 1/4 inch (6mm) of the plastic insulation from all four wire ends.
2. **Group and Twist:**
    - Twist the **two Red wires** together to make one thick "Positive" lead.
    - Twist the **two Black wires** together to make one thick "Negative/Ground" lead.
    - _Why?_ Using both wires for each side allows the system to carry more electrical current safely without the wires getting hot.

Step B: Connect to the Terminal Block

1. **Open the Terminals:** Use your screwdriver to turn the screws on top of the adapter counter-clockwise until the "jaws" inside the holes are fully open.
2. **Insert Wires:** Look for the **+** and **-** markings on the plastic of the adapter.
    - Insert the twisted **Red wires** into the **+** (Positive) side.
    - Insert the twisted **Black wires** into the **-** (Negative) side.
3. **Tighten:** Turn the screws clockwise until they firmly grip the wires. Give each wire a gentle tug to make sure they won't pull out. 

4. The Safe "Power-On" Sequence

To avoid sparks or damaging the sensitive Orin NX module, always follow this order:

1. **Connect to Board:** Plug the 4-pin Mini-Fit Jr. end of the CBG408 into the **P13 Power Header** on your Boson-22 board.
2. **Connect DC Jack:** Plug the round barrel from your `EA1951E-190` power supply into your newly attached female terminal adapter.
3. **Connect Wall Power:** Finally, plug the power supply's AC cord into the wall outlet.

Safety Check for Beginners

- **Polarity:** If you swap Red and Black, you will likely **destroy** your Jetson module instantly. Double-check that Red is in **+** and Black is in **-**.
- **Loose Strands:** Ensure no stray copper "hairs" from the red wires are touching the black wires (or vice versa). This causes a short circuit.
- **Cooling:** Do not power the system on for more than a few seconds unless you have already attached the fan/heatsink to the Orin NX module. It will overheat very quickly.



# Handling & Assembly Safety

- **Don't "Hot-Plug":** Never connect the CBG408 cable to the board while the 19V supply is already plugged into the wall and active. Connect the cable to the board first, then plug the power supply into the wall.
- **Cooling:** Since you have an **Orin NX 16GB (TE980M-A1)**, it will generate significant heat. Do not power it on without the fan/heatsink properly mounted and plugged into the **P10 Fan Header**.
- **ESD:** As mentioned, use a grounded metal desktop case or an ESD strap while handling these bare boards.