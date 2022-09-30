<?xml version="1.0"?>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns:xs="http://www.w3.org/2001/XMLSchema" xmlns="http://www.w3.org/1999/xhtml" version="1.0" exclude-result-prefixes="xs">

  <xsl:key name="Missing" match="/PGNDefinitions/MissingEnumerations/MissingAttribute" use="@Name" />

  <!--
    Some fields contain "word~text~" where text should be rendered
    as a subscript, ie. the html <sub> tag.
  -->
  <xsl:template name="replace-subscript">
    <xsl:param name="text" />
    <xsl:param name="replace" select="'~'"/>
    <xsl:choose>
      <xsl:when test="contains($text, $replace)">
        <xsl:value-of select="substring-before($text, $replace)" />
        <sub>
          <xsl:value-of select="substring-before(substring-after($text, $replace), $replace)" />
        </sub>
        <xsl:value-of select="substring-after(substring-after($text, $replace), $replace)" />
      </xsl:when>
      <xsl:otherwise>
        <xsl:value-of select="$text" />
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <xsl:template name="ConvertDecToHex">
    <xsl:param name="n" />
    <xsl:if test="$n > 0">
      <xsl:call-template name="ConvertDecToHex">
        <xsl:with-param name="n" select="floor($n div 16)" />
      </xsl:call-template>
      <xsl:choose>
        <xsl:when test="$n mod 16 = 10">A</xsl:when>
        <xsl:when test="$n mod 16 = 11">B</xsl:when>
        <xsl:when test="$n mod 16 = 12">C</xsl:when>
        <xsl:when test="$n mod 16 = 13">D</xsl:when>
        <xsl:when test="$n mod 16 = 14">E</xsl:when>
        <xsl:when test="$n mod 16 = 15">F</xsl:when>
        <xsl:otherwise>
          <xsl:value-of select="$n mod 16" />
        </xsl:otherwise>
      </xsl:choose>
    </xsl:if>
  </xsl:template>

  <xsl:template name="getFirstRepeatingField">
    <xsl:choose>
      <xsl:when test="RepeatingFields">
        <xsl:value-of select="FieldCount - RepeatingFields + 1"/>
      </xsl:when>
      <xsl:when test="RepeatingFieldSet2">
        <xsl:value-of select="FieldCount - RepeatingFieldSet2 - RepeatingFieldSet1 + 1"/>
      </xsl:when>
      <xsl:when test="RepeatingFieldSet1">
        <xsl:value-of select="FieldCount - RepeatingFieldSet1 + 1"/>
      </xsl:when>
      <xsl:otherwise>
        <xsl:value-of select="FieldCount + 1"/>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <xsl:template name="HandleRepeatingFields">
    <!-- 
      RepeatingFields is generated when there is one set of repeating fields without a repeat count.
      RepeatingFieldSet1 and RepeatingFieldSet2 only when the PGN has one or two repeating sets with 
      a field that says how often this is repeated. 
    -->

    <xsl:choose>
      <xsl:when test="RepeatingFields">
        <xsl:variable name="repeating" select="RepeatingFields"/>
        <xsl:variable name="startRepeat" select="FieldCount - $repeating + 1"/>
        The last <xsl:value-of select="$repeating"/> fields starting at field <xsl:value-of select="$startRepeat"/> repeat until the data is exhausted.
      </xsl:when>

      <xsl:when test="RepeatingFieldSet2">
        <xsl:variable name="repeatingSet1" select="RepeatingFieldSet1"/>
        <xsl:variable name="repeatingCount1" select="RepeatingFieldSet1CountField"/>
        <xsl:variable name="repeatingSet2" select="RepeatingFieldSet2"/>
        <xsl:variable name="repeatingCount2" select="RepeatingFieldSet2CountField"/>
        <xsl:variable name="startRepeat" select="FieldCount - $repeatingSet1 - $repeatingSet2 + 1"/>

        The <xsl:value-of select="$repeatingSet1"/> fields starting at field <xsl:value-of select="$startRepeat"/> are repeated <i>n</i> times, where <i>n</i> is determined by the value of field <xsl:value-of select="$repeatingCount1"/>.
        The <xsl:value-of select="$repeatingSet2"/> fields starting at field <xsl:value-of select="$startRepeat + $repeatingSet1"/> are repeated <i>n</i> times, where <i>n</i> is determined by the value of field <xsl:value-of select="$repeatingCount2"/>.
      </xsl:when>

      <xsl:when test="RepeatingFieldSet1">
        <xsl:variable name="repeatingSet1" select="RepeatingFieldSet1"/>
        <xsl:variable name="repeatingCount1" select="RepeatingFieldSet1CountField"/>
        <xsl:variable name="startRepeat" select="FieldCount - $repeatingSet1 + 1"/>

        The <xsl:value-of select="$repeatingSet1"/> fields starting at field <xsl:value-of select="$startRepeat"/> are repeated <i>n</i> times, where <i>n</i> is determined by the value of field <xsl:value-of select="$repeatingCount1"/>.
      </xsl:when>

    </xsl:choose>
  </xsl:template>

  <xsl:template name="pgn-list">
    <h2 id='pgn-list'>PGN list</h2>
    <xsl:for-each select="/PGNDefinitions/PGNs/*">
      <xsl:choose>
        <xsl:when test="./Fallback = 'true'">
          <h2>
            <xsl:attribute name="id">
              <xsl:value-of select="concat('pgn-', PGN)"/>
            </xsl:attribute>
            <xsl:value-of select="Description/text()"/>
          </h2>
          <p>
            <xsl:value-of select="Explanation/."/>
          </p>
        </xsl:when>
        <xsl:otherwise>
          <h3>
            <xsl:attribute name="id">
              <xsl:value-of select="concat('pgn-', PGN)"/>
            </xsl:attribute>
            <xsl:text>0x</xsl:text>
            <xsl:call-template name="ConvertDecToHex">
              <xsl:with-param name="n" select="PGN/." />
            </xsl:call-template>:
            PGN <xsl:value-of select="PGN/."/>
            - <xsl:value-of select="Description/text()"/>
          </h3>
          <xsl:if test="boolean(normalize-space(Fields/Field/Match))">
            <p>
              This PGN description applies when the following field(s) match:
              <table>
                <xsl:for-each select="Fields/Field/Match">
                  <tr><td><xsl:value-of select="../Name"/></td><td><xsl:value-of select="."/></td></tr>
                </xsl:for-each>
              </table>
            </p>
          </xsl:if>
          <p>
            <xsl:value-of select="Explanation/."/>
          </p>
          <p>
            <xsl:choose>
              <xsl:when test="Complete = 'false'">
                This PGN is not fully reverse engineered. Some aspects that are known to be missing/incorrect:
                <ul>
                  <xsl:for-each select="Missing/*">
                    <li>
                      <xsl:value-of select="key('Missing', .)/."/>
                    </li>
                  </xsl:for-each>
                </ul>
              </xsl:when>
              <xsl:otherwise>
                This PGN is believed to be (nearly) fully reverse engineered and seems to work in most practical purposes.
              </xsl:otherwise>
            </xsl:choose>
          </p>

          <xsl:variable name="startRepeat">
            <xsl:call-template name="getFirstRepeatingField" />
          </xsl:variable>
          <p>
            This
            <xsl:if test="Type = 'Single'"> single-frame </xsl:if>
            <xsl:if test="Type = 'Fast'"> fast-packet </xsl:if>
            PGN is
            <xsl:if test="MinLength">
              at least <xsl:value-of select="MinLength"/>
            </xsl:if>
            <xsl:if test="Length">
              <xsl:value-of select="Length"/>
            </xsl:if>
            bytes long and contains 
            <xsl:if test="$startRepeat = FieldCount + 1">
              <xsl:value-of select="FieldCount"/> fields.
            </xsl:if>
            <xsl:if test="$startRepeat != FieldCount + 1">
              at least <xsl:value-of select="$startRepeat + 1"/> fields.
            </xsl:if>

            <xsl:call-template name="HandleRepeatingFields"/>

            <xsl:if test="TransmissionInterval">
              It is normally transmitted every <xsl:value-of select="TransmissionInterval"/> milliseconds.
            </xsl:if>
            <xsl:if test="TransmissionIrregular = 'true'">
              It is not transmitted at a regular interval, but when data is available or it is requested.
            </xsl:if>
          </p>
          <table>
            <tr>
              <th> Field # </th>
              <th> Field Name </th>
              <th> Description </th>
              <th> Size (bits) </th>
              <th> Type </th>
              <th> Unit </th>
              <th> Lookup </th>
            </tr>
            <xsl:for-each select="Fields/*">
              <xsl:variable name="resolution" select="Resolution"/>
              <xsl:variable name="notone">
                <xsl:choose>
                  <xsl:when test="$resolution = '1'"/>
                  <xsl:otherwise>
                    <xsl:value-of select="$resolution"/><xsl:text> </xsl:text>
                  </xsl:otherwise>
                </xsl:choose>
              </xsl:variable>
              <tr>
                <xsl:if test="Order = $startRepeat">
                  <xsl:attribute name="class">startrepeating</xsl:attribute>
                </xsl:if>
                <td>
                  <xsl:if test="Order >= $startRepeat">
                    <xsl:attribute name="class">repeating</xsl:attribute>
                  </xsl:if>
                  <xsl:value-of select="Order"/>
                </td>
                <td> <xsl:value-of select="Name"/> </td>
                <td>
                  <xsl:if test="Match">
                    <xsl:value-of select="Match"/>:
                  </xsl:if>
                  <xsl:call-template name="replace-subscript">
                    <xsl:with-param name="text">
                      <xsl:value-of select="Description"/>
                    </xsl:with-param>
                  </xsl:call-template>
                </td>
                <td> <xsl:value-of select="BitLength"/> </td>
                <td>
                  <a>
                    <xsl:attribute name="href">
                      <xsl:value-of select="concat('#ft-', FieldType)"/>
                    </xsl:attribute>
                    <xsl:value-of select="FieldType"/>
                  </a>
                </td>
                <td>
                  <xsl:value-of select="$notone"/>
                  <xsl:value-of select="Unit"/>
                </td>
                <td>
                  <xsl:if test="LookupEnumeration">
                    <a>
                      <xsl:attribute name="href">
                        <xsl:value-of select="concat('#lookup-', LookupEnumeration)"/>
                      </xsl:attribute>
                      <xsl:value-of select="LookupEnumeration"/>
                    </a>
                  </xsl:if>
                  <xsl:if test="LookupBitEnumeration">
                    <a>
                      <xsl:attribute name="href">
                        <xsl:value-of select="concat('#lookupbit-', LookupBitEnumeration)"/>
                      </xsl:attribute>
                      <xsl:value-of select="LookupBitEnumeration"/>
                    </a>
                  </xsl:if>
                </td>
              </tr>
            </xsl:for-each>
          </table>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:for-each>
  </xsl:template>

  <xsl:template name="lookup-list">
    <h2 id='lookup-enumerations'>Lookup enumerations</h2>
    <xsl:for-each select="/PGNDefinitions/LookupEnumerations/*">
      <h3>
        <xsl:attribute name="id">
          <xsl:value-of select="concat('lookup-', @Name)"/>
        </xsl:attribute>
        <xsl:value-of select="@Name"/>
        (0 - <xsl:value-of select="@MaxValue"/>)
      </h3>
      <table>
        <tr>
          <th>Value</th>
          <th>Description</th>
        </tr>
        <xsl:for-each select="EnumPair">
          <tr>
            <td>
              <xsl:value-of select="@Value"/>
            </td>
            <td>
              <xsl:value-of select="@Name"/>
            </td>
          </tr>
        </xsl:for-each>
      </table>
    </xsl:for-each>
  </xsl:template>

  <xsl:template name="lookupbit-list">
    <h2 id='bitfield-enumerations'>Bitfield Lookup enumerations</h2>
    <xsl:for-each select="/PGNDefinitions/LookupBitEnumerations/*">
      <h3>
        <xsl:attribute name="id">
          <xsl:value-of select="concat('lookupbit-', @Name)"/>
        </xsl:attribute>
        <xsl:value-of select="@Name"/>
        (0 - 2^<xsl:value-of select="@MaxValue"/>-1)
      </h3>
      <table>
        <tr>
          <th>Bit</th>
          <th>Description</th>
        </tr>
        <xsl:for-each select="BitPair">
          <tr>
            <td>
              <xsl:value-of select="@Bit"/>
            </td>
            <td>
              <xsl:value-of select="@Name"/>
            </td>
          </tr>
        </xsl:for-each>
      </table>
    </xsl:for-each>
  </xsl:template>

  <xsl:template name="fieldtypes-list">
    <h2 id='field-types'>Field types</h2>

    All fields are of one of the following field types.

    <xsl:for-each select="/PGNDefinitions/FieldTypes/*">
      <h3>
        <xsl:attribute name="id">
          <xsl:value-of select="concat('ft-', @Name)"/>
        </xsl:attribute>
        <xsl:value-of select="@Name"/> - <xsl:value-of select="Description"/>
      </h3>
      <p> <xsl:value-of select="Comment"/> </p>
      <xsl:choose>
        <xsl:when test="Bits">
          <p>
            <xsl:value-of select="Bits"/> bits
            <xsl:choose>
              <xsl:when test="Signed = 'true'">
                signed
              </xsl:when>
              <xsl:otherwise>
                unsigned
              </xsl:otherwise>
            </xsl:choose>
            number
            <xsl:choose>
              <xsl:when test="RangeMin">
                with range
                <xsl:value-of select="RangeMin"/>
                to
                <xsl:value-of select="RangeMax"/>
                <xsl:value-of select="' '"/>
                <xsl:value-of select="Unit"/>
              </xsl:when>
            </xsl:choose>
            <xsl:choose>
              <xsl:when test="Resolution">
                in steps of 
                <xsl:value-of select="Resolution"/>
                <xsl:value-of select="' '"/>
                <xsl:value-of select="Unit"/>
              </xsl:when>
            </xsl:choose>
          </p>
        </xsl:when>
      </xsl:choose>
      <xsl:choose>
        <xsl:when test="EncodingDescription">
          <p>
            Encoding:
            <xsl:value-of select="EncodingDescription"/>
          </p>
        </xsl:when>
      </xsl:choose>
      <xsl:if test="URL">
        <p> 
          External website explanation: 
          <a>
            <xsl:attribute name="href">
              <xsl:value-of select="URL"/>
            </xsl:attribute>
            <xsl:value-of select="URL"/>
          </a>
        </p>
      </xsl:if>
      <xsl:choose>
        <xsl:when test="BaseFieldType">
          <p>
            This is a specific instance of a 
            <a>
              <xsl:attribute name="href">
                <xsl:value-of select="concat('#ft-', BaseFieldType)"/>
              </xsl:attribute>
              <xsl:value-of select="BaseFieldType"/> 
            </a>
            field.
          </p>
        </xsl:when>
      </xsl:choose>
    </xsl:for-each>
  </xsl:template>

  <xsl:template match="/">
    <xsl:apply-templates select="Response"/>
  </xsl:template>

  <xsl:template match="/">

    <xsl:variable name="license" select="/PGNDefinitions/License/text()"/>

    <html lang="en">
      <head>
        <meta charset="UTF-8"/>
        <meta name="description" content="CANBoat PGN documentation"/>
        <!-- <link rel="icon" type="image/gif" href="https://www.redwood.com/favicon.ico"/> -->
        <link rel="stylesheet" type="text/css" href="canboat.css"/>
        <script src="canboat.js"/>
        <link href="https://fonts.googleapis.com/css2?family=Noto+Sans:wght@300&amp;display=swap" rel="stylesheet"/>
      </head>
      <body>

        <div id="sidenav" class="sidenav">
          <a href="javascript:void(0)" class="closebtn" onclick="closeNav()">&#215;</a>
          <a href="#main">Top</a>
          <a href="#pgn-list">PGN list</a>
          <a href="#field-types">Field Types</a>
          <a href="#lookup-enumerations">Lookup enumerations</a>
          <a href="#bitfield-enumerations">Bitfield enumerations</a>
        </div>

        <div id="sidenav-closed" class="sidenav">
          <a href="javascript:void(0)" onclick="openNav()" id="hamburger">
            <span/>
            <span/>
            <span/>
          </a>
        </div>

        <div id="main">
          <p>
            The CANBoat project PGN documentation version
            <xsl:value-of select="/PGNDefinitions/Version/text()"/>.
          </p>
          <h3>Copyright</h3>
          <p class='xs'>
            <xsl:value-of select="/PGNDefinitions/Copyright"/>
          </p>

          <h2 id='frame-header'>ISO-11783 and NMEA2000 header</h2>

          An "Extended 29 bit identifier" CAN frame, as mandated by NMEA 2000, consists of a 39 bit header followed by (up to) 64 data bits. The header bits are:

          <table>
            <thead>
              <tr>
                <th>Bit</th><th>CAN</th><th>ISO 11783</th><th>Notes</th>
              </tr>
            </thead>
            <tbody>
              <tr><td> 1</td><td>SOF</td>  <td>SOF </td><td>Start of Frame</td> </tr>
              <tr><td> 2</td><td>ID 28</td><td>P 3 </td><td>Priority bit 3</td> </tr>
              <tr><td> 3</td><td>ID 27</td><td>P 2 </td><td>Priority bit 2</td> </tr>
              <tr><td> 4</td><td>ID 26</td><td>P 1 </td><td>Priority bit 1</td> </tr>
              <tr><td> 5</td><td>ID 25</td><td>R 1 </td><td>Reserved bit 1</td> </tr>
              <tr><td> 6</td><td>ID 24</td><td>DP  </td><td>Data page</td> </tr>
              <tr><td> 7</td><td>ID 23</td><td>PF 8</td><td>PDU format bit 8</td> </tr>
              <tr><td> 8</td><td>ID 22</td><td>PF 7</td><td>PDU format bit 7</td> </tr>
              <tr><td> 9</td><td>ID 21</td><td>PF 6</td><td>PDU format bit 6</td> </tr>
              <tr><td>10</td><td>ID 20</td><td>PF 5</td><td>PDU format bit 5</td> </tr>
              <tr><td>11</td><td>ID 19</td><td>PF 4</td><td>PDU format bit 4</td> </tr>
              <tr><td>12</td><td>ID 18</td><td>PF 3</td><td>PDU format bit 3</td> </tr>
              <tr><td>13</td><td>SRR  </td><td>SRR </td><td>Substitute Remote Request bit</td> </tr>
              <tr><td>14</td><td>IDE  </td><td>IDE </td><td>Identifier Extension bit</td> </tr>
              <tr><td>15</td><td>ID 17</td><td>PF 2</td><td>PDU format bit 2</td> </tr>
              <tr><td>16</td><td>ID 16</td><td>PF 1</td><td>PDU format bit 1</td> </tr>
              <tr><td>17</td><td>ID 15</td><td>PS 8</td><td>PDU specific bit 8</td> </tr>
              <tr><td>18</td><td>ID 14</td><td>PS 7</td><td>PDU specific bit 7</td> </tr>
              <tr><td>19</td><td>ID 13</td><td>PS 6</td><td>PDU specific bit 6</td> </tr>
              <tr><td>20</td><td>ID 12</td><td>PS 5</td><td>PDU specific bit 5</td> </tr>
              <tr><td>21</td><td>ID 11</td><td>PS 4</td><td>PDU specific bit 4</td> </tr>
              <tr><td>22</td><td>ID 10</td><td>PS 3</td><td>PDU specific bit 3</td> </tr>
              <tr><td>23</td><td>ID 9 </td><td>PS 2</td><td>PDU specific bit 2</td> </tr>
              <tr><td>24</td><td>ID 8 </td><td>PS 1</td><td>PDU specific bit 1</td> </tr>
              <tr><td>25</td><td>ID 7 </td><td>SA 8</td><td>Source Address bit 8</td> </tr>
              <tr><td>26</td><td>ID 6 </td><td>SA 7</td><td>Source Address bit 7</td> </tr>
              <tr><td>27</td><td>ID 5 </td><td>SA 6</td><td>Source Address bit 6</td> </tr>
              <tr><td>28</td><td>ID 4 </td><td>SA 5</td><td>Source Address bit 5</td> </tr>
              <tr><td>29</td><td>ID 3 </td><td>SA 4</td><td>Source Address bit 4</td> </tr>
              <tr><td>30</td><td>ID 2 </td><td>SA 3</td><td>Source Address bit 3</td> </tr>
              <tr><td>31</td><td>ID 1 </td><td>SA 2</td><td>Source Address bit 2</td> </tr>
              <tr><td>32</td><td>ID 0 </td><td>SA 1</td><td>Source Address bit 1</td> </tr>
              <tr><td>33</td><td>RTR  </td><td>RTR </td><td>Remote Tranmission Request bit</td> </tr>
              <tr><td>34</td><td>r 1  </td><td>r 1 </td><td>CAN reserved bit 1</td> </tr>
              <tr><td>35</td><td>r 0  </td><td>r 0 </td><td>CAN reserved bit 0</td> </tr>
              <tr><td>36</td><td>DLC 4</td><td>DLC 4</td><td>Data Length Code bit 4</td> </tr>
              <tr><td>37</td><td>DLC 3</td><td>DLC 3</td><td>Data Length Code bit 3</td> </tr>
              <tr><td>38</td><td>DLC 2</td><td>DLC 2</td><td>Data Length Code bit 2</td> </tr>
              <tr><td>39</td><td>DLC 1</td><td>DLC 1</td><td>Data Length Code bit 1</td> </tr>
            </tbody>
          </table>

          <p>
            For NMEA2000 the "R 1" bit is always 0, but in SAE J1939 it is not. J1939 calls this the "Extended Data Page" (EDP).
            In general, we don't care about these and the interfaces will generally only produce the 29 bits named "ID xx" in the above table. This is called the <i>CAN Id</i>.
          </p>

          <p>
            The source address and priority are easily derived from the above table: they form contiguous bits in the header. But the PGN and destination address are formed from the remaining (29 - 3 - 8) = 18 bits in a convoluted fashion.
            The algorithm is as follows:
            <ul>
              <li>
                Generate the values for PF (8 bits) and PS (8 bits) from their contiguous bits in the header. </li>
              <li>
                Generate a value RDP (2 bits) from the "R 1" and "DP" bits.</li>
              <li>
                If the value for PF is less than 240 (0xF0) this is called "PDU1" format. PS contains the destination address and the PGN is formed by <code>[RDP][PF]00000000</code>.</li>
              <li>
                Otherwise if PF is in range 240 - 255 this is called "PDU2" format. The destination is always 'global' (i.e. address 255) and the PGN is formed by <code>[RDP][PF][PS]</code>.</li>
            </ul>

            Note that this means that all PDU1 PGNs are 256 apart, as their lower 8 bits are always zero. Thus there are much fewer of them.
          </p>


          <h2 id='pgn-ranges'>PGN ranges</h2>

          <p>
            There are a number of PGN (Parameter Group Number) ranges. When expressed in decimal the ranges do not make much sense, but in hexadecimal they do -- which is explained above.
            The PGN ranges are divided up by PDU1/PDU2 but also by the fact that there is the need to have manufacture private PGNs, so some blocks have been carved out for this purpose.
            The ranges are:
          </p>
          <table>
            <thead>
              <tr><th>Range Hex</th><th>Range Dec</th><th>PDU</th><th>Step</th><th>Number of possible PGNs</th><th>Use</th><th>Framing</th></tr>
            </thead>
            <tbody>
              <tr><td><a href="#pgn-0">0xE800-0xEE00</a></td><td>59392 - 60928</td><td>PDU1</td><td>256</td><td>7</td><td>ISO 11783 (protocol)</td><td>Single frame</td></tr>
              <tr><td><a href="#pgn-61184">0xEF00</a></td><td>61184</td><td>PDU1</td><td></td><td>1</td><td>Manufacturer proprietary</td><td>Single frame</td></tr>
              <tr><td><a href="#pgn-61440">0xF000-0xFEFF</a></td><td>61440 - 65279</td><td>PDU2</td><td>1</td><td>3840</td><td>Standardized</td><td>Single frame</td></tr>
              <tr><td><a href="#pgn-65280">0xFF00-0xFFFF</a></td><td>65280 - 65535</td><td>PDU2</td><td>1</td><td>256</td><td>Manufacturer proprietary</td><td>Single frame</td></tr>
              <tr><td><a href="#pgn-65536">0x1ED00-0x1EE00</a></td><td>126208 - 126464</td><td>PDU1</td><td>256</td><td>2</td><td>Standardized (protocol)</td><td>Single frame</td></tr>
              <tr><td><a href="#pgn-126720">0x1EF00</a></td><td>126720</td><td>PDU1</td><td></td><td>1</td><td>Manufacturer proprietary</td><td>Fast packet</td></tr>
              <tr><td><a href="#pgn-126976">0x1F000-0x1FEFF</a></td><td>126976 - 130815</td><td>PDU2</td><td>1</td><td>3840</td><td>Standardized</td><td>Mixed single/fast</td></tr>
              <tr><td><a href="#pgn-130816">0x1FF00-0x1FFFF</a></td><td>130816 - 131071</td><td>PDU2</td><td>1</td><td>256</td><td>Manufacturer proprietary</td><td>Fast packet</td></tr>
            </tbody>
          </table>
          <p>
            <b>Note:</b> There are some missing ranges in the above table: Apparently no PGN is used in range 0x0000-0xE700 or 0x10000-0x1EC00. The reason is not obvious to me. Maybe someone can enlighten us?
          </p>

          <h2 id='packet-framing'>Packet framing</h2>

          <p>
            NMEA 2000 messages that are 8 bytes or less can be transmitted in a single CAN frame. For messages of 9 or more bytes there
            is an ISO 11783 defined method called Transport Protocol that can be used to transmit up to 1785 bytes. 
            See <a href="#pgn-60416">PGN 60416</a>. This is <i>not</i> generally used though. What is used
            is an alternative method with less overhead and less complexity for the sender. This is called <i>fast</i> packet framing.
          </p>

          <p>
            In fast packet framing, the first packet contains two protocol bytes and six data bytes. Following packets contain one
            protocol byte and seven data bytes. Up to 32 packets can be used for a single message so the total maxing data length
            is 6 + 31 * 7 = 223 bytes.
            The first byte in all frames contains a sequence counter in the high 3 bits and a frame counter in the lower 5 bits.
            The second byte in the first frame contains the total number of frames that will be sent.
            As there is no way to acknowledge or deny reception, if a message is missed the receiver will have to wait for the next
            transmission of the message.
          </p>

          <p>
            It is unclear to us whether the standard mandates that frames are transmitted in order. According to <a href="https://www.yachtd.com/news/strict_frame_order.html">Yacht Devices</a>
            there are (older?) devices that transmit out-of-order as this is easier to program with certain CAN chips, and various new MFDs ignore out-of-order packet transmissions.
            As of v4.0.0 the CANBoat analyzer program will analyze out-of-order frames, although it means it has to heuristically determine when a packet is complete.
          </p>

          <xsl:call-template name="pgn-list"/>
          <xsl:call-template name="fieldtypes-list"/>
          <xsl:call-template name="lookup-list"/>
          <xsl:call-template name="lookupbit-list"/>

        </div>

      </body>
    </html>
  </xsl:template>


</xsl:stylesheet>
