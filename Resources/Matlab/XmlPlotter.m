xDoc = xmlread('PTracker-Kinect_1.xml');

xRoot = xDoc.getDocumentElement;
schema = char(xRoot.getAttribute('xsi:noNamespaceSchemaLocation'));

infoLabel = 'object';
infoCbk = '';
itemFound = false;

% Find a deep list of all listitem elements.
allListItems = xDoc.getElementsByTagName('object');

% Note that the item list index is zero-based.
for k = 0:allListItems.getLength-1
   thisListItem = allListItems.item(k);
   childNode = thisListItem.getFirstChild;
   
   while ~isempty(childNode)
      %Filter out text, comments, and processing instructions.
      if childNode.getNodeType == childNode.ELEMENT_NODE
         % Assume that each element has a single
         % org.w3c.dom.Text child.
         
         if childNode.getTagName == 'box'
           disp(childNode.getAttributes.item(2))
           disp(childNode.getAttributes.item(3))
           disp(childNode.getAttributes.item(0))
           disp(childNode.getAttributes.item(1))
           disp('*******************************************************')
         end
      end  % End IF
      childNode = childNode.getNextSibling;
   end  % End WHILE

   if itemFound
      break;
   else
      infoCbk = '';
   end
end  % End FOR

disp(sprintf('Item "%s" has a callback of "%s".', infoLabel, infoCbk))