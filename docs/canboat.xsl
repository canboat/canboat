<?xml version="1.0"?>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns:xs="http://www.w3.org/2001/XMLSchema" xmlns="http://www.w3.org/1999/xhtml" version="1.0" exclude-result-prefixes="xs">

  <xsl:key name="Missing" match="/PGNDefinitions/Missing/MissingAttribute" use="@Name" />

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

  <xsl:template name="pgn-list">
    <h2 id='pgn-list'>PGN list</h2>
    <xsl:for-each select="/PGNDefinitions/PGNs/*">
      <h3>
        <xsl:attribute name="id">
          <xsl:value-of select="concat('pgn-', PGN)"/>
        </xsl:attribute>
        PGN <xsl:value-of select="PGN/."/>
        - <xsl:value-of select="Description/text()"/>
      </h3>
        <p>
          <xsl:value-of select="Explanation/."/>
        </p>
        <p>
          <xsl:choose>
            <xsl:when test="Complete = 'false'">
              This PGN is not fully reverse engineered. Some aspects that are known to be missing/incorrect:
              <ul>
                <xsl:for-each select="Missing">
                  <li>
                    <xsl:value-of select="key('Missing', MissingAttribute)/."/>
                  </li>
                </xsl:for-each>
              </ul>
            </xsl:when>
            <xsl:otherwise>
              This PGN is believed to be (nearly) fully reverse engineered and seems to work in most practical purposes.
            </xsl:otherwise>
          </xsl:choose>
        </p>
        <p>
          This <xsl:value-of select="Type"/> PGN contains <xsl:value-of select="Length"/> bytes.
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
          <xsl:variable name="resolution">
            <xsl:value-of select="Resolution"/>
          </xsl:variable>
          <xsl:variable name="notone">
            <xsl:choose>
              <xsl:when test="$resolution = '1'"/>
              <xsl:otherwise>
                <xsl:value-of select="$resolution"/><xsl:text> </xsl:text>
              </xsl:otherwise>
            </xsl:choose>
          </xsl:variable>
          <tr>
            <td> <xsl:value-of select="Order"/> </td>
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
              <xsl:choose>
                <xsl:when test="LookupEnumeration">
                  <a>
                    <xsl:attribute name="href">
                      <xsl:value-of select="concat('#lookup-', LookupEnumeration)"/>
                    </xsl:attribute>
                    <xsl:value-of select="LookupEnumeration"/>
                  </a>
                </xsl:when>
              </xsl:choose>
              <xsl:choose>
                <xsl:when test="LookupBitEnumeration">
                  <a>
                    <xsl:attribute name="href">
                      <xsl:value-of select="concat('#lookupbit-', LookupBitEnumeration)"/>
                    </xsl:attribute>
                    <xsl:value-of select="LookupBitEnumeration"/>
                  </a>
                </xsl:when>
              </xsl:choose>
            </td>
          </tr>
        </xsl:for-each>
      </table>
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

    <xsl:variable name="license">
      <xsl:value-of select="/PGNDefinitions/License/text()"/>
    </xsl:variable>

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
