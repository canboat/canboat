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

  <xsl:template name="pgn-list">
    <h2 id='pgn-list'>PGN list</h2>
    <xsl:for-each select="/PGNDefinitions/PGNs/*">
      <xsl:choose>
        <xsl:when test="./Fallback = 'true'">
          <h2>
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
          <xsl:variable name="repeating" select="RepeatingFields"/>
          <xsl:variable name="fieldCount" select="count(Fields/*)"/>
          <xsl:variable name="startRepeat" select="$fieldCount - $repeating"/>
          <p>
            This <xsl:value-of select="Type"/> PGN contains <xsl:value-of select="Length"/> bytes and <xsl:value-of select="$fieldCount"/> fields.
            <xsl:if test="$repeating > 0">
              The last <xsl:value-of select="$repeating"/> fields starting at field <xsl:value-of select="$startRepeat"/> repeat until the data is exhausted.
            </xsl:if>
          </p>
          <table>
            <tr>
              <th> Order </th>
              <th> Name </th>
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
                <xsl:if test="Order = $startRepeat + 1">
                  <xsl:attribute name="class">startrepeating</xsl:attribute>
                </xsl:if>
                <td>
                  <xsl:if test="Order > $startRepeat">
                    <xsl:attribute name="class">repeating</xsl:attribute>
                  </xsl:if>
                  <xsl:value-of select="Order"/>
                </td>
                <td> <xsl:value-of select="Name"/> </td>
                <td>
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

          <xsl:call-template name="pgn-list"/>
          <xsl:call-template name="fieldtypes-list"/>
          <xsl:call-template name="lookup-list"/>
          <xsl:call-template name="lookupbit-list"/>

        </div>

      </body>
    </html>
  </xsl:template>


</xsl:stylesheet>
